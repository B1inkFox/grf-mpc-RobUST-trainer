using UnityEngine;
using System;
using System.Threading;

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

    // Vicon SDK access
    private object viconClient;

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
        forcePlateThread.Start();
        
        Debug.Log($"ForcePlateManager initialized successfully. Sampling at {samplingRate_Hz} Hz.");
        return true;
    }
    
    /// <summary>
    /// Try to initialize the Vicon SDK
    /// </summary>
    private bool InitializeViconSDK()
    {
        // Try to load the Vicon assembly
        var assembly = System.Reflection.Assembly.Load("ViconDataStreamSDK_DotNET");
        if (assembly == null)
        {
            Debug.LogError("Failed to load ViconDataStreamSDK_DotNET assembly. Make sure all DLLs are correctly placed in the Plugins/x86_64 folder.");
            return false;
        }
        
        // Create the client dynamically
        Type clientType = assembly.GetType("ViconDataStreamSDK.DotNET.Client");
        if (clientType == null)
        {
            Debug.LogError("Failed to find Client type in Vicon SDK assembly.");
            return false;
        }
        
        viconClient = Activator.CreateInstance(clientType);
        
        // Connect to the local Vicon DataStream server - using the exact format from the example
        string connectionString = $"localhost:{serverPort}";
        Debug.Log($"Connecting to Vicon DataStream at {connectionString}...");
        
        // Connect to Vicon server
        var connectMethod = clientType.GetMethod("Connect");
        var result = connectMethod.Invoke(viconClient, new object[] { connectionString });
        
        // Check result
        var resultProperty = result.GetType().GetProperty("Result");
        object resultValue = resultProperty.GetValue(result);
        
        if (!resultValue.ToString().Equals("Success"))
        {
            Debug.LogError($"Failed to connect to Vicon DataStream: {resultValue}");
            return false;
        }
        
        // Enable the required data types
        clientType.GetMethod("EnableDeviceData").Invoke(viconClient, null);
        
        // Get force plate count
        var getForcePlateCountMethod = clientType.GetMethod("GetForcePlateCount");
        var countResult = getForcePlateCountMethod.Invoke(viconClient, null);
        var countProperty = countResult.GetType().GetProperty("ForcePlateCount");
        numForcePlates = (int)Convert.ToUInt32(countProperty.GetValue(countResult));
        
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
        // Cache method info objects for better performance
        Type clientType = viconClient.GetType();
        var getFrameMethod = clientType.GetMethod("GetFrame");
        var getGlobalForceVectorMethod = clientType.GetMethod("GetGlobalForceVector");
        var getGlobalCentreOfPressureMethod = clientType.GetMethod("GetGlobalCentreOfPressure");
        
        // Precise timing using high-resolution Stopwatch
        double exactIntervalTicks = (double)System.Diagnostics.Stopwatch.Frequency / samplingRate_Hz;
        long targetIntervalTicks = (long)Math.Round(exactIntervalTicks);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + targetIntervalTicks;
        
        while (isRunning)
        {
            // Get a new frame from Vicon
            getFrameMethod.Invoke(viconClient, null);
            
            // Process each force plate directly
            for (uint i = 0; i < numForcePlates; i++)
            {
                // Get force in global coordinate system
                var forceResult = getGlobalForceVectorMethod.Invoke(viconClient, new object[] { i });
                var forceVectorProperty = forceResult.GetType().GetProperty("ForceVector");
                double[] forceVector = (double[])forceVectorProperty.GetValue(forceResult);
                
                // Get center of pressure
                var copResult = getGlobalCentreOfPressureMethod.Invoke(viconClient, new object[] { i });
                var copVectorProperty = copResult.GetType().GetProperty("CentreOfPressure");
                double[] copVector = (double[])copVectorProperty.GetValue(copResult);
                
                // Thread-safe update directly to the array
                lock (dataLock)
                {
                    forcePlateDataArray[i] = new ForcePlateData(
                        new Vector3(
                            (float)forceVector[0],
                            (float)forceVector[1],
                            (float)forceVector[2]
                        ),
                        new Vector3(
                            (float)copVector[0],
                            (float)copVector[1],
                            (float)copVector[2]
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
            // Disconnect using reflection
            var disconnectMethod = viconClient.GetType().GetMethod("Disconnect");
            disconnectMethod.Invoke(viconClient, null);
        }
    }
}
