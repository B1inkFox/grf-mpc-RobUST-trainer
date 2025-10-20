using UnityEngine;
using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;

/// <summary>
/// Handles threaded TCP communication with LabVIEW, continuously sending the latest tension data.
/// </summary>
public class LabviewTcpCommunicator : MonoBehaviour
{
    [Header("Network Settings")]
    public string serverAddress = "127.0.0.1";
    public int serverPort = 8053;

    [Header("Motor Mapping")]
    [Tooltip("The motor number corresponding to each cable tension. Order must match CableTensionPlanner output.")]
    public int[] motorNumbers = new int[] { -1, -1, -1, -1 }; // Default configuration

    // Network components
    private TcpClient tcpClient;
    private NetworkStream networkStream;
    
    // Threading
    private Thread sendThread;
    private double sendFrequency_Hz = 500.0; 
    private volatile bool isRunning = false;
    private readonly object dataLock = new object();
    private volatile char controlModeCode = 'O';

    // Current data to send - pre-allocated during initialization
    private double[] tensions;

    public bool IsConnected { get; private set; } = false;

    // Cache for send thread to avoid allocations (only tensions need copying)
    private double[] sendTensions;

    /// <summary>
    /// Initializes the TCP communicator with the cable configuration.
    /// Called by RobotController in the correct dependency order.
    /// </summary>
    /// <param name="numCables">Number of cables from the tension planner</param>
    /// <returns>True if initialization succeeded, false otherwise</returns>
    public bool Initialize(int numCables)
    {
        if (motorNumbers == null || motorNumbers.Length != numCables)
        {
            Debug.LogError($"LabviewTcpCommunicator: Motor numbers array must have {numCables} elements to match cable count.", this);
            return false;
        }

        // Pre-allocate arrays based on cable count
        tensions = new double[numCables];
        sendTensions = new double[numCables];
        
        Debug.Log($"TCP Communicator initialized for {numCables} cables with motors: [{string.Join(", ", motorNumbers)}]");
        return true;
    }

    /// <summary>
    /// Updates only the tension values (zero-allocation, zero-check real-time performance).
    /// </summary>
    public void UpdateTensionSetpoint(double[] newTensions)
    {
        // No checks - arrays are guaranteed to be the right size at startup
        lock (dataLock)
        {
            Array.Copy(newTensions, tensions, tensions.Length);
        }
    }

    /// <summary>
    /// Switches outgoing packets to closed-loop control mode.
    /// </summary>
    public void SetClosedLoopControl()
    {
        controlModeCode = 'C';
    }

    /// <summary>
    /// Switches outgoing packets to open-loop control mode.
    /// </summary>
    public void SetOpenLoopControl()
    {
        controlModeCode = 'O';
    }

    public async void ConnectToServer()
    {
        try
        {
            tcpClient = new TcpClient();
            UnityEngine.Debug.Log($"Connecting to {serverAddress}:{serverPort}...");
            
            await tcpClient.ConnectAsync(serverAddress, serverPort);
            networkStream = tcpClient.GetStream();
            tcpClient.NoDelay = true; // Disable Nagle's algorithm
            
            IsConnected = true;
            isRunning = true;
            sendThread = new Thread(SendLoop) { IsBackground = true };
            sendThread.Priority = System.Threading.ThreadPriority.Highest;
            sendThread.Start();
            
            UnityEngine.Debug.Log("Connected to LabVIEW server.");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Connection failed: {e.Message}");
        }
    }

    /// <summary>
    /// Background thread that continuously sends data at precise 1kHz.
    /// </summary>
    private void SendLoop()
    {
        if (networkStream == null || tcpClient == null || !tcpClient.Connected)
        {
            isRunning = false;
            IsConnected = false;
            return;
        }

        // Use double precision and proper rounding to avoid truncation errors
        double exactIntervalTicks = (double)System.Diagnostics.Stopwatch.Frequency / sendFrequency_Hz;
        long targetIntervalTicks = (long)Math.Round(exactIntervalTicks);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + targetIntervalTicks;
        
        while (isRunning)
        {
            // Get thread-safe copy of tension data
            lock (dataLock)
            {
                Array.Copy(tensions, sendTensions, tensions.Length);
            }
            // Send data
            string packet = FormatPacket(motorNumbers, sendTensions);
            byte[] data = Encoding.ASCII.GetBytes(packet);
            networkStream.Write(data, 0, data.Length);

            // Precise timing: wait until next target time
            long timeUntilNext = nextTargetTime - System.Diagnostics.Stopwatch.GetTimestamp();
            double sleepMs = (double)timeUntilNext * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
            
            if (sleepMs > 0.1)
            { // Use SpinWait for sub-millisecond precision (0.1-1.0ms range)
                SpinWait.SpinUntil(() => System.Diagnostics.Stopwatch.GetTimestamp() >= nextTargetTime);
            } // else: For sleepMs <= 0.1, just continue

            // Advance to next target time
            nextTargetTime += targetIntervalTicks;
            // Drift compensation: if we're behind, reset to maintain frequency
            long currentTime = System.Diagnostics.Stopwatch.GetTimestamp();
            if (nextTargetTime <= currentTime)
            {
                // We're behind - skip ahead to maintain frequency
                nextTargetTime = currentTime + targetIntervalTicks;
            }
        }
    }

    /// <summary>
    /// Formats the tension data into the LabVIEW protocol.
    /// </summary>
    private string FormatPacket(int[] motors, double[] tensions)
    {
        var sb = new StringBuilder();
        sb.Append(controlModeCode);
        sb.Append(',');
        sb.Append(motors.Length);
        
        for (int i = 0; i < motors.Length; i++)
        {
            sb.Append($",{motors[i]},{tensions[i]:F6}");
        }
        
        // Use double precision for timestamp calculation
        double timestampMs = (double)System.Diagnostics.Stopwatch.GetTimestamp() * 1000.0 / (double)System.Diagnostics.Stopwatch.Frequency;
        sb.Append($",{timestampMs}\r\n");
        return sb.ToString();
    }

    /// <summary>
    /// Manually disconnect from the server and clean up resources.
    /// </summary>
    public void Disconnect()
    {
        if (!IsConnected) return;
        
        isRunning = false;
        sendThread?.Join(500);
        networkStream?.Close();
        tcpClient?.Close();
        IsConnected = false;
        
        UnityEngine.Debug.Log("Disconnected from LabVIEW server.");
    }

    void OnApplicationQuit()
    {
        Disconnect();
    }
}
