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

    public enum CONTROL_MODE { OFF, TRANSPARENT, MPC, IMPEDANCE }
    public volatile CONTROL_MODE currentControlMode = CONTROL_MODE.OFF;

    [Header("Robot Geometry Configuration")]
    [Tooltip("Number of cables in the system.")]
    public int numCables = 8;

    [Tooltip("Measured thickness of the chest in the anterior-posterior direction [m].")]
    public float chestAPDistance = 0.2f;

    [Tooltip("Measured width of the chest in the medial-lateral direction [m].")]
    public float chestMLDistance = 0.3f;

    [Tooltip("User body mass [kg].")]
    public float userMass = 70.0f;
    [Tooltip("User shoulder width [m].")]
    public float userShoulderWidth = 0.4f;
    [Tooltip("User trunk height (hip to shoulder) [m].")]
    public float userHeight = 0.5f;

    private RobUSTDescription robotDescription;
    private MPCSolver controller;
    private CableTensionPlanner tensionPlanner;

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
        robotDescription = RobUSTDescription.Create(numCables, chestAPDistance, chestMLDistance, 
                                                    userMass, userShoulderWidth, userHeight);
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
            tcpCommunicator.SetClosedLoopControl();
        }

        // Initialize Controller Here
        controller = new MPCSolver(robotDescription, 0.05, 10);
        tensionPlanner = new CableTensionPlanner(robotDescription);

        controllerThread = new Thread(controlLoop)
        {
            Name = "Robot Controller Main",
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.AboveNormal
        };

        isRunning = true;
        controllerThread.Start();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.O)) { currentControlMode = CONTROL_MODE.OFF; }
        if (Input.GetKeyDown(KeyCode.T)) { currentControlMode = CONTROL_MODE.TRANSPARENT; }
        if (Input.GetKeyDown(KeyCode.M)) { currentControlMode = CONTROL_MODE.MPC; }
        if (Input.GetKeyDown(KeyCode.I)) { currentControlMode = CONTROL_MODE.IMPEDANCE; }
    }

    /// <summary>
    ///  This control loop is the main "driver" of one or more `Controller` instances. 
    ///  It needs to grab the latest tracker data, update visuals, compute cable tensions, and send commands to LabVIEW.
    ///  a `Controller` functions more like a `Control Policy/Solver` here, with RobotController managing the data flow.
    /// </summary>
    private void controlLoop()
    {
        double ctrl_freq = 100.0;
        Span<double> motor_tension_command = stackalloc double[14];
        double[] solver_tensions = null;
        double4x4 framePose = ToDouble4x4(robot_frame_tracker.PoseMatrix);
        double4x4 frameInv = math.fastinverse(framePose);
        
        SensorFilter filter_10Hz = new SensorFilter(ctrl_freq, 10.0);

        double system_frequency = System.Diagnostics.Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / system_frequency;
        long intervalTicks = (long)(system_frequency / ctrl_freq);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + intervalTicks;
        long lastLoopTick = System.Diagnostics.Stopwatch.GetTimestamp();

        while (isRunning)
        {
            long loopStartTick = System.Diagnostics.Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            trackerManager.GetEndEffectorTrackerData(out TrackerData rawEndEffectorData);
            trackerManager.GetCoMTrackerData(out TrackerData rawComData);
            forcePlateManager.GetNetForcePlateData(out ForcePlateData netFPData);
            
            double4x4 eePose_RF = math.mul(frameInv, ToDouble4x4(rawEndEffectorData.PoseMatrix));
            double4x4 comPose_RF = math.mul(frameInv, ToDouble4x4(rawComData.PoseMatrix));
            
            filter_10Hz.Update(comPose_RF, netFPData);

            switch (currentControlMode)
            {
                case CONTROL_MODE.OFF:
                    motor_tension_command.Clear();
                    break;
                case CONTROL_MODE.TRANSPARENT:
                    Wrench goalWrench; // zero wrench
                    solver_tensions = tensionPlanner.CalculateTensions(eePose_RF, goalWrench);
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    break;
                case CONTROL_MODE.MPC:
                    ForcePlateData filteredFPData = new ForcePlateData(filter_10Hz.FilteredGRF, filter_10Hz.FilteredCoP);
                    controller.UpdateState(eePose_RF, comPose_RF, filter_10Hz.LinearVelocity, filter_10Hz.AngularVelocity, filteredFPData);
                    solver_tensions = controller.computeNextControl();
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    break;
                case CONTROL_MODE.IMPEDANCE:
                    motor_tension_command.Clear(); // Placeholder
                    break;
            }

            tcpCommunicator.UpdateTensionSetpoint(motor_tension_command);

            visualizer.SetTrackerData(rawComData, rawEndEffectorData, robot_frame_tracker);

            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime) { } // BURN wait

            nextTargetTime += intervalTicks;
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime) nextTargetTime = now + intervalTicks; // drift correction
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
        double4x4 frameInv = math.fastinverse(framePose);
        return math.mul(frameInv, eePose);
    }

    // Local helper (keep near RobotController; same as already used elsewhere)
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
    /// Maps solver tensions to the 14-motor driver array using the active configuration.
    /// </summary>
    private void MapTensionsToMotors(double[] solverResult, Span<double> output)
    {
        output.Clear();
        int count = robotDescription.SolverToMotorMap.Length;
        for (int i = 0; i < count; i++)
        {
            int motorIndex = robotDescription.SolverToMotorMap[i];
            output[motorIndex] = solverResult[i];
        }
    }

    // private class for recursively filtering sensor data
    private class SensorFilter
    {
        private readonly double dt, alpha;
        private double3 comPositionPrev;
        private double3x3 R_comPrev;
        private bool isFirstUpdate = true;

        public double3 LinearVelocity { get; private set; }
        public double3 AngularVelocity { get; private set; }
        public double3 FilteredGRF { get; private set; }
        public double3 FilteredCoP { get; private set; }

        public SensorFilter(double frequency, double cutoffHz)
        {
            dt = 1.0 / frequency;
            double tau = 1.0 / (2.0 * math.PI * cutoffHz);
            alpha = dt / (tau + dt);
        }

        public void Update(double4x4 comPose_RF, in ForcePlateData rawForceData)
        {
            double3 comPosition = comPose_RF.c3.xyz;
            double3x3 R_com = new double3x3(comPose_RF.c0.xyz, comPose_RF.c1.xyz, comPose_RF.c2.xyz);

            if (isFirstUpdate)
            {
                comPositionPrev = comPosition;
                R_comPrev = R_com;
                FilteredGRF = rawForceData.Force;
                FilteredCoP = rawForceData.CenterOfPressure;
                isFirstUpdate = false;
                return;
            }

            double3 rawLinVel = (comPosition - comPositionPrev) / dt;
            double3x3 R_diff = math.mul(R_com, math.transpose(R_comPrev));
            double3 rawAngVel = new double3(
                R_diff.c1.z - R_diff.c2.y,
                R_diff.c2.x - R_diff.c0.z,
                R_diff.c0.y - R_diff.c1.x
            ) / (2.0 * dt);

            LinearVelocity = (alpha * rawLinVel) + ((1.0 - alpha) * LinearVelocity);
            AngularVelocity = (alpha * rawAngVel) + ((1.0 - alpha) * AngularVelocity);
            FilteredGRF = (alpha * rawForceData.Force) + ((1.0 - alpha) * FilteredGRF);
            FilteredCoP = (alpha * rawForceData.CenterOfPressure) + ((1.0 - alpha) * FilteredCoP);

            comPositionPrev = comPosition;
            R_comPrev = R_com;
        }
    }
    

}
