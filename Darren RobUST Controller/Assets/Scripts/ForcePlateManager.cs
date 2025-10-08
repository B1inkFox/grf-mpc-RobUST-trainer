using UnityEngine;
using System;
using System.Threading;
using ViconDataStreamSDK.CSharp;

/// <summary>
/// Manages data from the Vicon force plates using "ServerPush" mode. Vicon Box triggers
/// syncing with unity program at exactly 100 hz for deterministic sampling of ground reaction forces and center of pressure.
/// </summary>
public class ForcePlateManager : MonoBehaviour
{    
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
        forcePlateThread.Priority = System.Threading.ThreadPriority.AboveNormal;
        forcePlateThread.Start();
        
        Debug.Log($"ForcePlateManager initialized successfully on port {serverPort}.");
        return true;
    }
    
    /// <summary>
    /// Initialize the Vicon SDK using the Unity plugin
    /// </summary>
    private bool InitializeViconSDK()
    {
        if (numForcePlates <= 0)
        {
            Debug.LogError("Force Plate Count in Inspector is not set to a positive integer.");
            return false;
        }

        // Allocate data arrays based on force plate count
        forcePlateDataArray = new ForcePlateData[numForcePlates];
        for (int i = 0; i < numForcePlates; i++)
        {
            forcePlateDataArray[i] = new ForcePlateData(Vector3.zero, Vector3.zero);
        }

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
        while (isRunning)
        {
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
    public ForcePlateData[] GetForcePlateData()
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
