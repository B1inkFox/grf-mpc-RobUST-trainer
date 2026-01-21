using UnityEngine;
using Unity.Mathematics;
using Unity.Profiling;

using System;
using System.Threading;

/// <summary>
/// Main controller for the cable-driven robot.
/// This class centralizes the robot's state management, coordinating trackers,
/// force plates, physics calculations, and communication with LabVIEW.
/// </summary>
public class RobotController : MonoBehaviour
{
    static readonly ProfilerCounterValue<long> s_WorkloadNs = new(RobotProfiler.Workloads, "Controller Workload", ProfilerMarkerDataUnit.TimeNanoseconds);
    static readonly ProfilerCounterValue<long> s_IntervalNs = new(RobotProfiler.Intervals, "Controller Execution Interval", ProfilerMarkerDataUnit.TimeNanoseconds);

    [Header("Module References")]
    [Tooltip("The TrackerManager instance that provides tracker data.")]
    public TrackerManager trackerManager;

    [Tooltip("The ForcePlateManager instance for reading force plate data.")]
    public ForcePlateManager forcePlateManager;

    [Tooltip("The LabviewTcpCommunicator instance for sending motor commands.")]
    public LabviewTcpCommunicator tcpCommunicator;

    [Header("Visualization")]
    [Tooltip("Handles tracker/camera visuals. Keeps RobotController logic-only.")]
    public RobotVisualizer visualizer;

    [Header("Control Settings")]
    [Tooltip("Enable or disable sending commands to LabVIEW.")]
    public bool isLabviewControlEnabled = true;

    [Header("Robot Geometry Configuration")]
    [Tooltip("Number of cables in the system.")]
    public int numCables = 8;

    [Tooltip("Measured thickness of the chest in the anterior-posterior direction [m].")]
    public float chestAPDistance = 0.2f;

    [Tooltip("Measured width of the chest in the medial-lateral direction [m].")]
    public float chestMLDistance = 0.3f;

    private RobUSTDescription robotDescription;
    private BaseController<Wrench> controller;

    // Static frame reference captured only at startup to prevent drift
    private TrackerData robot_frame_tracker;
    private Thread controllerThread;
    private volatile bool isRunning = false;


    private void Start()
    {
        using (System.Diagnostics.Process p = System.Diagnostics.Process.GetCurrentProcess())
        {
            p.PriorityClass = System.Diagnostics.ProcessPriorityClass.High;
        }

        if (!ValidateModules())
        {
            enabled = false; // Disable this script if modules are missing from inspector
            return;
        }

        // Validate geometry configuration
        if (chestAPDistance <= 0 || chestMLDistance <= 0)
        {
            Debug.LogError("Chest dimensions must be positive values.", this);
            enabled = false;
            return;
        }

        // Create RobUST description (single allocation at startup)
        robotDescription = RobUSTDescription.Create(numCables, chestAPDistance, chestMLDistance);
        Debug.Log($"RobUST description created: {numCables} cables, AP={chestAPDistance}m, ML={chestMLDistance}m");

        if (!trackerManager.Initialize())
        {
            Debug.LogError("Failed to initialize TrackerManager.", this);
            enabled = false;
            return;
        }
        if (!forcePlateManager.Initialize())
        {
            Debug.LogError("Failed to initialize ForcePlateManager.", this);
            // dont disable program if no force plates
        }
        if (!tcpCommunicator.Initialize())
        {
            Debug.LogError("Failed to initialize LabviewTcpCommunicator.", this);
            enabled = false;
            return;
        }

        // Capture static frame reference to prevent drift during operation
        Thread.Sleep(100); // Brief pause for tracking stability
        trackerManager.GetFrameTrackerData(out robot_frame_tracker);

        // Initialize visualizer now that frame pose is available
        ReadOnlySpan<Vector3> pulleyPositions = robotDescription.FramePulleyPositionsVec3;
        if (!visualizer.Initialize(robot_frame_tracker.PoseMatrix, pulleyPositions))
        {
            Debug.LogError("Failed to initialize RobotVisualizer.", this);
            enabled = false;
            return;
        }

        if (isLabviewControlEnabled)
        {
            tcpCommunicator.ConnectToServer();
        }

        // Initialize Controller Here
        controller = new MPCSolver(0.05, 10, 4);

        controllerThread = new Thread(controlLoop)
        {
            Name = "Robot Controller Main",
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.AboveNormal
        };

        isRunning = true;
        controllerThread.Start();
    }

    /// <summary>
    ///  This control loop is the main "driver" of a `Controller` instance. 
    ///  It needs to grab the latest tracker data, update visuals, compute cable tensions, and send commands to LabVIEW.
    ///  a `Controller` functions more like a `Control Policy/Solver` here, with RobotController managing the data flow.
    /// </summary>
    private void controlLoop()
    {
        TrackerData rawComData, rawEndEffectorData;
        Span<double> tension_command = stackalloc double[14];

        double frequency = System.Diagnostics.Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / frequency;
        long intervalTicks = (long)(frequency / 100.0); // Control loop at 100 Hz
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + intervalTicks;
        long lastLoopTick = System.Diagnostics.Stopwatch.GetTimestamp();
        
        while (isRunning)
        {
            long loopStartTick = System.Diagnostics.Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            // 1. Get the latest raw tracker data (in the arbitrary, RIGHT-HANDED OpenVR/Vive coordinate system).
            trackerManager.GetCoMTrackerData(out rawComData);
            trackerManager.GetEndEffectorTrackerData(out rawEndEffectorData);

            // 2. Cache tracker data for visualization (thread-safe, applied in visualizer's Update())
            visualizer.SetTrackerData(rawComData, rawEndEffectorData, robot_frame_tracker);

            //Here we parse the control effort that we obtained from the controller, to be implemented
            // Compute cable tensions using the controller
            // map tensions to tension_command array



            // Send the calculated tensions to LabVIEW
            tcpCommunicator.SetClosedLoopControl();
            tcpCommunicator.UpdateTensionSetpoint(tension_command);

            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
            
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime)
            {
                // (BURN CPU cycles to hold timing precision)
                // prevents OS scheduler from yielding the thread 
            }
            nextTargetTime += intervalTicks;
            
            // If we are late (processing took > 10 ms), reset the pacer
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime)
            {
                nextTargetTime = now + intervalTicks;
            }
        }
    }

    /// <summary>
    /// Helper method to get chest tracker's position relative to the static frame reference.
    /// Useful for manually recording pulley positions during calibration.
    /// </summary>
    /// <returns>Position relative to frame tracker </returns>
    public double4x4 GetEEPoseRelativeToFrame()
    {
        // Grab EE pose (UnityEngine.Matrix4x4 stored in TrackerData)
        trackerManager.GetEndEffectorTrackerData(out TrackerData eeData);

        // Convert both matrices to Unity.Mathematics.double4x4
        double4x4 eePose = ToDouble4x4(eeData.PoseMatrix);
        double4x4 framePose = ToDouble4x4(robot_frame_tracker.PoseMatrix);

        // Compute relative pose: frame^-1 * ee
        double4x4 frameInv = math.inverse(framePose);
        return math.mul(frameInv, eePose);
    }

    // Local helper (keep near RobotController; same as you already used elsewhere)
    private static double4x4 ToDouble4x4(in Matrix4x4 m)
    {
        return new double4x4(
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }


    // Validates that all required module references are assigned in the Inspector.
    // returns True if all modules are assigned, false otherwise.
    private bool ValidateModules()
    {
        bool allValid = true;
        if (trackerManager == null) { Debug.LogError("Module not assigned in Inspector: trackerManager", this); allValid = false; }
        if (forcePlateManager == null) { Debug.LogError("Module not assigned in Inspector: forcePlateManager", this); allValid = false; }
        if (tcpCommunicator == null) { Debug.LogError("Module not assigned in Inspector: tcpCommunicator", this); allValid = false; }
        if (visualizer == null) { Debug.LogError("Visualizer not assigned in Inspector: visualizer", this); allValid = false; }

        return allValid;
    }

    private void OnDestroy()
    {
        // Clean shutdown of threaded components.
        // TrackerManager handles its own shutdown via its OnDestroy method.
        isRunning = false;
        tcpCommunicator?.Disconnect();
    }


    /// <summary>
    /// Maps the 8-DOF solver tensions to the 14-DOF motor driver array.
    /// Zero allocations.
    /// </summary>
    // Change return type to void. Pass the destination 'output' as an argument.
    private void MapTensionsToMotors(double[] solverResult, Span<double> output)
    {
        // No allocations here. Just direct memory writing.
        output[0] = 0;
        output[1] = solverResult[6];
        output[2] = 0;
        output[3] = solverResult[2];
        output[4] = solverResult[1];
        output[5] = 0;
        output[6] = solverResult[5];
        output[7] = solverResult[4];
        output[8] = 0;
        output[9] = solverResult[0];
        output[10] = solverResult[3];
        output[11] = 0;
        output[12] = solverResult[7];
        output[13] = 0;
    }
    

}
