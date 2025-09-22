using UnityEngine;
using System;
using System.Threading;
using ViconDataStreamSDK.DotNET;

/// <summary>
/// Manages data from the Vicon force plates using a dedicated thread
/// for deterministic sampling of ground reaction forces and center of pressure.
/// </summary>
public class ForcePlateManager : MonoBehaviour
{
    [Header("Connection Settings")]
    [Tooltip("Port of the Vicon DataStream server")]
    public int serverPort = 801;
    
    [Header("Performance Settings")]
    [Tooltip("Target sampling rate in Hz")]
    public double samplingRate_Hz = 100.0;

    private int numForcePlates;
    
    // Thread-related fields
    private Thread forcePlateThread;
    private volatile bool isRunning = false;
    private bool isConnected = false;
    
    // Thread-safe data storage with locking
    private readonly object dataLock = new object();
    private ForcePlateData[] forcePlateDataArray;

    // Direct reference to Vicon client
    private Client viconClient;

    /// <summary>
    /// Initializes the force plate manager.
    /// Called by RobotController in the correct dependency order.
    /// </summary>
    /// <returns>True if initialization succeeded, false otherwise</returns>
    public bool Initialize()
    {
        // Initialize the Vicon SDK
        if (!InitializeViconSDK())
        {
            Debug.LogError("Failed to initialize Vicon force plate connection.");
            return false;
        }
        
        // Start the sampling thread
        isRunning = true;
        forcePlateThread = new Thread(ForcePlateSamplingLoop);
        forcePlateThread.IsBackground = true;
        forcePlateThread.Priority = ThreadPriority.AboveNormal;
        forcePlateThread.Start();
        
        Debug.Log($"ForcePlateManager initialized successfully. Sampling at {samplingRate_Hz} Hz.");
        return true;
    }
    
    /// <summary>
    /// Try to initialize the Vicon SDK
    /// </summary>
    private bool InitializeViconSDK()
    {
        // Create the client directly
        viconClient = new Client();
        
        // Connect to the local Vicon DataStream server
        string connectionString = $"localhost:{serverPort}";
        Debug.Log($"Connecting to Vicon DataStream at {connectionString}...");
        
        // Connect to Vicon server
        Output_Connect result = viconClient.Connect(connectionString);
        
        if (result.Result != Result.Success)
        {
            Debug.LogError($"Failed to connect to Vicon DataStream: {result.Result}");
            return false;
        }
        
        // Enable the required data types
        viconClient.EnableDeviceData();
        
        // Get force plate count
        Output_GetForcePlateCount countResult = viconClient.GetForcePlateCount();
        numForcePlates = (int)countResult.ForcePlateCount;
        
        if (numForcePlates <= 0)
        {
            Debug.LogError("No force plates detected in Vicon system.");
            return false;
        }
        
        // Allocate data arrays based on detected force plate count
        forcePlateDataArray = new ForcePlateData[numForcePlates];
        for (int i = 0; i < numForcePlates; i++)
        {
            forcePlateDataArray[i] = new ForcePlateData(Vector3.zero, Vector3.zero);
        }
        
        isConnected = true;
        Debug.Log($"Successfully connected to Vicon DataStream. Detected {numForcePlates} force plates.");
        return true;
    }
    
    /// <summary>
    /// High-frequency sampling loop running in dedicated background thread.
    /// </summary>
    private void ForcePlateSamplingLoop()
    {
        // Precise timing using high-resolution Stopwatch
        double exactIntervalTicks = (double)System.Diagnostics.Stopwatch.Frequency / samplingRate_Hz;
        long targetIntervalTicks = (long)Math.Round(exactIntervalTicks);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + targetIntervalTicks;
        
        while (isRunning)
        {
            // Get a new frame from Vicon
            viconClient.GetFrame();
            
            // Process each force plate directly
            for (uint i = 0; i < numForcePlates; i++)
            {
                // Get force in global coordinate system
                Output_GetGlobalForceVector forceResult = viconClient.GetGlobalForceVector(i);
                
                // Get center of pressure
                Output_GetGlobalCentreOfPressure copResult = viconClient.GetGlobalCentreOfPressure(i);
                
                // Thread-safe update directly to the array
                lock (dataLock)
                {
                    forcePlateDataArray[i] = new ForcePlateData(
                        new Vector3(
                            (float)forceResult.ForceVector[0],
                            (float)forceResult.ForceVector[1],
                            (float)forceResult.ForceVector[2]
                        ),
                        new Vector3(
                            (float)copResult.CentreOfPressure[0],
                            (float)copResult.CentreOfPressure[1],
                            (float)copResult.CentreOfPressure[2]
                        )
                    );
                }
            }
            
            // Precise timing control
            long currentTime = System.Diagnostics.Stopwatch.GetTimestamp();
            if (nextTargetTime > currentTime)
            {
                // Use SpinWait for sub-millisecond precision
                SpinWait.SpinUntil(() => System.Diagnostics.Stopwatch.GetTimestamp() >= nextTargetTime);
            }

            // Advance to next target time with drift compensation
            nextTargetTime += targetIntervalTicks;
            currentTime = System.Diagnostics.Stopwatch.GetTimestamp();
            if (nextTargetTime <= currentTime)
            {
                nextTargetTime = currentTime + targetIntervalTicks;
            }
        }
    }

    /// <summary>
    /// Gets the force plate data for a specific plate.
    /// Thread-safe access.
    /// </summary>
    public ForcePlateData GetForcePlateData(int plateIndex = 0)
    {
        lock (dataLock)
        {
            if (plateIndex < 0 || plateIndex >= numForcePlates)
            {
                Debug.LogWarning($"Invalid force plate index: {plateIndex}");
                return new ForcePlateData(Vector3.zero, Vector3.zero);
            }
            return new ForcePlateData(
                forcePlateDataArray[plateIndex].Force, 
                forcePlateDataArray[plateIndex].CenterOfPressure
            );
        }
    }
    
    /// <summary>
    /// Gets a copy of the force plate data array for all plates.
    /// Thread-safe access.
    /// </summary>
    public ForcePlateData[] GetAllForcePlateData()
    {
        ForcePlateData[] result = new ForcePlateData[numForcePlates];
        lock (dataLock)
        {
            Array.Copy(forcePlateDataArray, result, numForcePlates);
        }
        return result;
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
