using UnityEngine;
using UnityEngine.InputSystem;
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
    private bool isForcePlateEnabled = true;

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
    [Tooltip("User height (feet to head) [m].")]
    public float userHeight = 1.5f;

    private RobUSTDescription robotDescription;
    private MPCSolver controller;
    private ImpedanceController impedanceController;
    private CableTensionPlanner tensionPlanner;

    // Static frame reference captured only at startup to prevent drift
    private TrackerData robot_frame_tracker;
    private Thread controllerThread;
    private volatile bool isRunning = false;

    // Global Reference Trajectory
    private RBState[] Xref_global;
    private long trajectoryIndex = 0; // Current index in controlLoop

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

        // Initialize Global Reference Trajectory
        Xref_global = new RBState[2000]; // 20 seconds buffer
        // stable standing
        RBState staticPoint = new RBState(
            new double3(0, 0.75, 0), 
            new double3(0, 0, math.PI/2), 
            new double3(0, 0, 0), 
            new double3(0, 0, 0)
        );
        for (int i = 0; i < Xref_global.Length; i++) Xref_global[i] = staticPoint;

        // Initialize Modules
        tcpCommunicator.Initialize();
        if (!trackerManager.Initialize())
        {
            Debug.LogError("Failed to initialize TrackerManager.", this);
            enabled = false;
            return;
        }
        if (!forcePlateManager.Initialize(robotDescription))
        {
            Debug.LogWarning("Failed to initialize ForcePlateManager.", this);
            isForcePlateEnabled = false;
        }
        
        trackerManager.GetFrameTrackerData(out robot_frame_tracker); // IMPORTANT snapshot frame tracker
        if (!visualizer.Initialize(robotDescription))
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

        // Finally Initialize Controllers and begin control thread
        controller = new MPCSolver(robotDescription, 0.05, 10);
        impedanceController = new ImpedanceController();
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
        if (Keyboard.current == null) return;

        if (Keyboard.current.oKey.wasPressedThisFrame) { currentControlMode = CONTROL_MODE.OFF; trajectoryIndex = 0; }
        if (Keyboard.current.tKey.wasPressedThisFrame) { currentControlMode = CONTROL_MODE.TRANSPARENT; }
        if (Keyboard.current.mKey.wasPressedThisFrame) { if (isForcePlateEnabled) currentControlMode = CONTROL_MODE.MPC; }
        if (Keyboard.current.iKey.wasPressedThisFrame) { currentControlMode = CONTROL_MODE.IMPEDANCE; }
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
            double4x4 eePose_RF = math.mul(frameInv, ToDouble4x4(rawEndEffectorData.PoseMatrix));
            double4x4 comPose_RF = math.mul(frameInv, ToDouble4x4(rawComData.PoseMatrix));

            forcePlateManager.GetForcePlateData(0, out ForcePlateData fp0);
            forcePlateManager.GetForcePlateData(1, out ForcePlateData fp1);
            double3 netForce = fp0.Force + fp1.Force;
            double3 netCoP = double3.zero;
            if (math.abs(netForce.z) > 1e-3) // Handle division by zero
                netCoP = (fp0.CoP * fp0.Force.z + fp1.CoP * fp1.Force.z) / netForce.z;
            ForcePlateData netFPData = new ForcePlateData(netForce, netCoP);
            
            filter_10Hz.Update(comPose_RF, eePose_RF, netFPData);

            switch (currentControlMode)
            {
                case CONTROL_MODE.OFF:
                    motor_tension_command.Clear();
                    break;
                case CONTROL_MODE.TRANSPARENT:
                    Wrench zeroWrench = new Wrench(double3.zero, double3.zero);
                    solver_tensions = tensionPlanner.CalculateTensions(eePose_RF, zeroWrench);
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    trajectoryIndex++;
                    break;
                case CONTROL_MODE.MPC:
                    ForcePlateData filteredFPData = new ForcePlateData(filter_10Hz.FilteredGRF, filter_10Hz.FilteredCoP);
                    controller.UpdateReferenceTrajectory(Xref_global.AsSpan(), (int)trajectoryIndex);
                    controller.UpdateState(eePose_RF, comPose_RF, filter_10Hz.CoMLinearVelocity, filter_10Hz.CoMAngularVelocity, filteredFPData);
                    solver_tensions = controller.computeNextControl();
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    trajectoryIndex++;
                    break;
                case CONTROL_MODE.IMPEDANCE:
                    int safeIdx = (trajectoryIndex < Xref_global.Length) ? (int)trajectoryIndex : Xref_global.Length - 1;
                    RBState target = ComputeEERefFromCoMRef(Xref_global[safeIdx], comPose_RF, eePose_RF);

                    impedanceController.UpdateState(eePose_RF, filter_10Hz.EELinearVelocity, filter_10Hz.EEAngularVelocity, target);
                    Wrench goalWrench = impedanceController.computeNextControl();
                    Debug.Log($"Impedance Control Wrench: F=({goalWrench.Force}), T=({goalWrench.Torque})");
                    solver_tensions = tensionPlanner.CalculateTensions(eePose_RF, goalWrench);
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    trajectoryIndex++;
                    break;
            }

            tcpCommunicator.UpdateTensionSetpoint(motor_tension_command);
            visualizer.PushState(comPose_RF, eePose_RF, fp0.CoP, fp0.Force, fp1.CoP, fp1.Force);

            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime) { } // BURN wait

            nextTargetTime += intervalTicks;
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime) nextTargetTime = now + intervalTicks; // drift correction
        }
    }

    /// <summary>
    /// Computes the corresponding End-Effector reference state from a CoM reference state.
    /// Uses the current body-frame offset r_local = R_curr^T * (p_ee - p_com)
    /// and rotates it by the reference orientation to support full rigid body kinematics.
    /// </summary>
    private RBState ComputeEERefFromCoMRef(RBState comRefState, double4x4 curr_comPose, double4x4 curr_eePose)
    {
        // 1. Calculate constant offset in body frame: r_local = R_com^T * (p_ee - p_com)
        double3 p_diff_world = curr_eePose.c3.xyz - curr_comPose.c3.xyz;
        double3x3 R_curr = new double3x3(curr_comPose.c0.xyz, curr_comPose.c1.xyz, curr_comPose.c2.xyz);
        double3 r_local = math.mul(math.transpose(R_curr), p_diff_world);

        // 2. Compute Rotation Matrix for Reference State (Euler ZYX)
        double3 th = comRefState.th; // (x:phi, y:theta, z:psi)
        
        // Build R_ref = Rz(psi) * Ry(theta) * Rx(phi) manually (Column-Major)
        double cz = math.cos(th.z), sz = math.sin(th.z);
        double cy = math.cos(th.y), sy = math.sin(th.y);
        double cx = math.cos(th.x), sx = math.sin(th.x);

        double3x3 R_z = new double3x3(new double3(cz, sz, 0), new double3(-sz, cz, 0), new double3(0, 0, 1));
        double3x3 R_y = new double3x3(new double3(cy, 0, -sy), new double3(0, 1, 0), new double3(sy, 0, cy));
        double3x3 R_x = new double3x3(new double3(1, 0, 0), new double3(0, cx, sx), new double3(0, -sx, cx));
        
        double3x3 R_ref = math.mul(R_z, math.mul(R_y, R_x));

        // 3. Compute EE Reference terms
        double3 r_ref_global = math.mul(R_ref, r_local);
        
        return new RBState(
            comRefState.p + r_ref_global,                          // p_ee_ref
            comRefState.th,                                        // Orientation (Shared)
            comRefState.v + math.cross(comRefState.w, r_ref_global), // v_ee_ref = v_com + w x r
            comRefState.w                                          // Angular Velocity (Shared)
        );
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
        
        // CoM State
        private double3 comPositionPrev;
        private double3x3 R_comPrev;
        
        // End Effector State
        private double3 eePositionPrev;
        private double3x3 R_eePrev;

        private bool isFirstUpdate = true;

        // outputs
        public double3 CoMLinearVelocity { get; private set; }
        public double3 CoMAngularVelocity { get; private set; }
        
        public double3 EELinearVelocity { get; private set; }
        public double3 EEAngularVelocity { get; private set; }

        public double3 FilteredGRF { get; private set; }
        public double3 FilteredCoP { get; private set; }

        public SensorFilter(double frequency, double cutoffHz)
        {
            dt = 1.0 / frequency;
            double tau = 1.0 / (2.0 * math.PI * cutoffHz);
            alpha = dt / (tau + dt);
        }

        public void Update(double4x4 comPose_RF, double4x4 eePose_RF, in ForcePlateData rawForceData)
        {
            double3 comPosition = comPose_RF.c3.xyz;
            double3x3 R_com = new double3x3(comPose_RF.c0.xyz, comPose_RF.c1.xyz, comPose_RF.c2.xyz);
            
            double3 eePosition = eePose_RF.c3.xyz;
            double3x3 R_ee = new double3x3(eePose_RF.c0.xyz, eePose_RF.c1.xyz, eePose_RF.c2.xyz);

            if (isFirstUpdate)
            {
                comPositionPrev = comPosition;
                R_comPrev = R_com;
                
                eePositionPrev = eePosition;
                R_eePrev = R_ee;

                FilteredGRF = rawForceData.Force;
                FilteredCoP = rawForceData.CoP;
                isFirstUpdate = false;
                return;
            }

            // --- CoM Velocities ---
            double3 rawCoMLinVel = (comPosition - comPositionPrev) / dt;
            
            // Angular velocity from rotation difference: R_diff approx I + skew(omega * dt)
            double3x3 R_com_diff = math.mul(R_com, math.transpose(R_comPrev));
            double3 rawCoMAngVel = new double3(
                R_com_diff.c1.z - R_com_diff.c2.y,
                R_com_diff.c2.x - R_com_diff.c0.z,
                R_com_diff.c0.y - R_com_diff.c1.x
            ) / (2.0 * dt);

            CoMLinearVelocity = (alpha * rawCoMLinVel) + ((1.0 - alpha) * CoMLinearVelocity);
            CoMAngularVelocity = (alpha * rawCoMAngVel) + ((1.0 - alpha) * CoMAngularVelocity);

            // --- End Effector Velocities ---
            double3 rawEELinVel = (eePosition - eePositionPrev) / dt;
            
            double3x3 R_ee_diff = math.mul(R_ee, math.transpose(R_eePrev));
            double3 rawEEAngVel = new double3(
                R_ee_diff.c1.z - R_ee_diff.c2.y,
                R_ee_diff.c2.x - R_ee_diff.c0.z,
                R_ee_diff.c0.y - R_ee_diff.c1.x
            ) / (2.0 * dt);

            EELinearVelocity = (alpha * rawEELinVel) + ((1.0 - alpha) * EELinearVelocity);
            EEAngularVelocity = (alpha * rawEEAngVel) + ((1.0 - alpha) * EEAngularVelocity);

            // --- Force Plates ---
            FilteredGRF = (alpha * rawForceData.Force) + ((1.0 - alpha) * FilteredGRF);
            FilteredCoP = (alpha * rawForceData.CoP) + ((1.0 - alpha) * FilteredCoP);

            // Update Previous States
            comPositionPrev = comPosition;
            R_comPrev = R_com;
            eePositionPrev = eePosition;
            R_eePrev = R_ee;
        }
    }
    

}
