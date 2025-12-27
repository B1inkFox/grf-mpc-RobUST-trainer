using UnityEngine;
using System;
using System.Net.Sockets;
using System.Threading;

/// <summary>
/// Handles threaded TCP communication with LabVIEW, continuously sending the latest tension data.
/// Optimized for Zero-Allocation and "Soft" Real-Time on Windows.
/// </summary>
public class LabviewTcpCommunicator : MonoBehaviour
{
    [Header("Network Settings")]
    public string serverAddress = "10.0.0.62";
    public int serverPort = 8053;

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

    // --- NEW: Zero-Allocation Buffers ---
    private double[] sendTensions; // Thread-local copy of data
    private byte[] sendBuffer;     // Raw bytes to send to TCP
    private char[] formatBuffer;   // Temp buffer for double->string conversion

    public bool IsConnected { get; private set; } = false;

    /// <summary>
    /// Initializes the TCP communicator with the cable configuration.
    /// Called by RobotController in the correct dependency order.
    /// </summary>
    public bool Initialize()
    {
        // Pre-allocate arrays based on cable count
        tensions = new double[14];
        sendTensions = new double[14];

        // NEW: Allocate buffers once. 
        // Size calc: 1 byte (Mode) + 14 * (1 comma + ~9 chars) + 2 newline = ~150 bytes. 
        // We give 512 for safety.
        sendBuffer = new byte[512];
        formatBuffer = new char[32]; // Enough for one double "0.123456"
        
        return true;
    }

    /// <summary>
    /// Updates only the tension values (zero-allocation, zero-check real-time performance).
    /// </summary>
    public void UpdateTensionSetpoint(double[] newTensions)
    {
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
            // Crucial: This allows us to bully other threads, but we still need the while loop below
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
    /// Background thread that continuously sends data at precise 500Hz.
    /// </summary>
    private void SendLoop()
    {
        if (networkStream == null || tcpClient == null || !tcpClient.Connected)
        {
            isRunning = false;
            IsConnected = false;
            return;
        }

        // Timing constants
        double frequency = System.Diagnostics.Stopwatch.Frequency;
        long intervalTicks = (long)(frequency / sendFrequency_Hz);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + intervalTicks;
        
        while (isRunning)
        {
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime)
            {
                // BURN cycles to hold the core.
                // We avoid Thread.SpinWait() here to prevent OS yielding.
            }

            // Get thread-safe copy of tension data
            lock (dataLock)
            {
                Array.Copy(tensions, sendTensions, tensions.Length);
            }

            int bytesToSend = FillPacketBuffer(sendTensions, sendBuffer);
            try 
            {
                networkStream.Write(sendBuffer, 0, bytesToSend);
            }
            catch (Exception) 
            {
                isRunning = false; // Stop if socket dies
                break;
            }

            // TIMING ADVANCE & DRIFT CORRECTION
            nextTargetTime += intervalTicks;

            // If we are late (processing took > 2ms), reset the pacer
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime)
            {
                nextTargetTime = now + intervalTicks;
            }
        }
    }

    /// <summary>
    /// Fills the byte array directly with ASCII data.
    /// Returns the number of bytes written.
    /// Allocates ZERO memory (no new Strings, no new Arrays).
    /// </summary>
    private int FillPacketBuffer(double[] values, byte[] buffer)
    {
        int pos = 0;

        // A. Write Control Mode
        buffer[pos++] = (byte)controlModeCode;

        // B. Write Tensions
        for (int i = 0; i < values.Length; i++)
        {
            buffer[pos++] = (byte)',';

            // ZERO-ALLOC NUMBER FORMATTING
            // TryFormat writes the number into 'formatBuffer' (char[]) without creating a String.
            // Requirement: Unity Project Settings > Player > Api Compatibility Level = .NET Standard 2.1
            if (values[i].TryFormat(formatBuffer, out int charsWritten, "F6"))
            {
                // Manually copy chars to bytes (ASCII)
                for (int k = 0; k < charsWritten; k++)
                {
                    buffer[pos++] = (byte)formatBuffer[k];
                }
            }
            else
            {
                // Fallback (Should ideally never happen with a large enough formatBuffer)
                buffer[pos++] = (byte)'0';
            }
        }

        // C. Write Newline
        buffer[pos++] = (byte)'\r';
        buffer[pos++] = (byte)'\n';

        return pos;
    }

    public void Disconnect()
    {
        if (!IsConnected) return;
        
        isRunning = false;
        // Wait a bit for the loop to finish its current spin
        sendThread?.Join(100); 
        
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