using UnityEngine;
using Unity.Mathematics; 
using Unity.Profiling;

using System;
using System.Threading;
using ViconDataStreamSDK.CSharp;

/// <summary>
/// Manages data from the Vicon force plates using "ServerPush" mode. Vicon Box triggers
/// syncing with unity program at exactly 100 hz for deterministic sampling of ground reaction forces and center of pressure.
/// </summary>
public class ForcePlateManager : MonoBehaviour
{    
    static readonly ProfilerCounterValue<long> s_WorkloadNs = new(RobotProfiler.Workloads, "ForcePlate Manager Workload", ProfilerMarkerDataUnit.TimeNanoseconds);
    static readonly ProfilerCounterValue<long> s_IntervalNs = new(RobotProfiler.Intervals, "ForcePlate Manager Interval", ProfilerMarkerDataUnit.TimeNanoseconds);

    [Tooltip("Hard-set number of force plates")]
    public int numForcePlates = 2;

    // Thread-related fields
    private Thread forcePlateThread;
    private volatile bool isRunning = false;
    private bool isConnected = false;
    private string serverPort = "801";
        
    // Thread-safe data storage with locking
    private readonly object dataLock = new object();
    private ForcePlateData[] forcePlateDataArray;

    private Client viconClient;

    private ForcePlateCalibrator calib;
        
    /// <summary>
    /// Initializes the force plate manager.
    /// Called by RobotController in the correct dependency order.
    /// </summary>
    /// <returns>True if initialization succeeded, false otherwise</returns>
    public bool Initialize(RobUSTDescription robotDescription)
    {
        // Ensure storage is ready even if SDK fails later
        if (numForcePlates <= 0)
        {
            Debug.LogError("Force Plate Count invalid.");
            return false;
        }
        forcePlateDataArray = new ForcePlateData[numForcePlates];
        calib = new ForcePlateCalibrator(robotDescription);
        
        // Initialize the Vicon SDK
        if (!InitializeViconSDK()) return false;

        // Start the sampling thread
        isRunning = true;
        forcePlateThread = new Thread(ForcePlateSamplingLoop)
        {
            IsBackground = true,
            Name = "Force Plate Thread",
            Priority = System.Threading.ThreadPriority.AboveNormal
        };
        forcePlateThread.Start();

        Debug.Log($"ForcePlateManager initialized successfully on port {serverPort}.");
        return true;
    }
    
    /// <summary>
    /// Initialize the Vicon SDK using the Unity plugin
    /// </summary>
    private bool InitializeViconSDK()
    {
        // Create the client directly
        viconClient = new Client();
        string connectionString = $"localhost:{serverPort}";
        Debug.Log($"Connecting to Vicon DataStream at {connectionString}...");

        // Attemp connection with Vicon Nexus Program
        if (viconClient.Connect(connectionString).Result != Result.Success)
        {
            Debug.LogError("Vicon didn't connect!");
            return false;
        }

        viconClient.SetStreamMode(StreamMode.ServerPush);
        // Need to get a frame before enabling device data
        viconClient.GetFrame();
        viconClient.EnableDeviceData();
        viconClient.GetFrame();
                
        isConnected = true;
        Debug.Log($"Successfully connected to Vicon DataStream");
        return true;
    }
    
    /// <summary>
    /// High-frequency sampling loop running in dedicated background thread.
    /// </summary>
    private void ForcePlateSamplingLoop()
    {
        double3 rawForce, rawCop;
        double3 finalForce, finalCop;

        double frequency = System.Diagnostics.Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / frequency;
        long lastLoopTick = System.Diagnostics.Stopwatch.GetTimestamp();

        while (isRunning)
        {
            viconClient.GetFrame(); // Blocks until Vicon pushes a frame

            long loopStartTick = System.Diagnostics.Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            // Process each force plate directly
            for (int i = 0; i < numForcePlates; i++)
            {
                // Get force & CoP in global coordinate system
                Output_GetGlobalForceVector forceResult = viconClient.GetGlobalForceVector((uint)i);
                Output_GetGlobalCentreOfPressure copResult = viconClient.GetGlobalCentreOfPressure((uint)i);

                rawForce = new double3(forceResult.ForceVector[0], forceResult.ForceVector[1], forceResult.ForceVector[2]);
                rawCop = new double3(copResult.CentreOfPressure[0], copResult.CentreOfPressure[1], copResult.CentreOfPressure[2]);

                // Transform immediately once
                calib.ProjectForce(in rawForce, out finalForce);
                calib.ProjectPosition(in rawCop, out finalCop);

                ForcePlateData plateData = new ForcePlateData(finalForce, finalCop);

                lock (dataLock)
                {
                    forcePlateDataArray[i] = plateData;
                }
            }

            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
        }
    }

    /// <summary>
    /// Gets the force plate data for a specific plate with zero allocations.
    /// Thread-safe access. Returns fully calibrated robot-frame data.
    /// </summary>
    public void GetForcePlateData(int plateIndex, out ForcePlateData data)
    {
        lock (dataLock)
        {
            if (plateIndex < 0 || plateIndex >= numForcePlates)
            {
                Debug.LogWarning($"Invalid force plate index: {plateIndex}");
                data = new ForcePlateData();
                return;
            }
            
            // Data is already transformed in the thread loop
            data = forcePlateDataArray[plateIndex];
        }
    }

    public void GetNetForcePlateData(out ForcePlateData netData)
    {
        Span<ForcePlateData> snapshot = stackalloc ForcePlateData[numForcePlates];
        lock (dataLock)
        {
            int copyLen = math.min(numForcePlates, forcePlateDataArray.Length);
            new ReadOnlySpan<ForcePlateData>(forcePlateDataArray, 0, copyLen).CopyTo(snapshot);
        }

        // Perform computation on local stack data (thread-safe, no lock needed)
        double3 netForce = double3.zero;
        double3 weightedCoPSum = double3.zero;
        double totalVerticalForce = 0.0;

        for (int i = 0; i < numForcePlates; i++)
        {
            ref var plate = ref snapshot[i];
            netForce += plate.Force;

            // Weight CoP by vertical force (Z-axis)
            double fz = math.abs(plate.Force.z);
            weightedCoPSum += plate.CoP * fz;
            totalVerticalForce += fz;
        }

        double3 netCoP = totalVerticalForce > 1e-3 ? weightedCoPSum / totalVerticalForce : double3.zero;
        netData = new ForcePlateData(netForce, netCoP);
    }  

    /// <summary>
    /// Clean up resources on application quit
    /// </summary>
    private void OnDestroy()
    {
        isRunning = false;
        if (forcePlateThread != null && forcePlateThread.IsAlive)
        {
            forcePlateThread.Join(500); // Wait up to 500ms for clean exit
        }
        
        if (isConnected && viconClient != null)
        {
            // Disconnect directly
            viconClient.Disconnect();
        }
    }
}
