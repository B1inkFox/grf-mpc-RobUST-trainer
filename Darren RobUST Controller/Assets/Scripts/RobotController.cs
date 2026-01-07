using UnityEngine;
using Unity.Mathematics;
using System;

/// <summary>
/// Main controller for the cable-driven robot.
/// This class centralizes the robot's state management, coordinating trackers,
/// force plates, physics calculations, and communication with LabVIEW.
/// </summary>
public class RobotController : MonoBehaviour
{
    [Header("Module References")]
    [Tooltip("The TrackerManager instance that provides tracker data.")]
    public TrackerManager trackerManager;

    [Tooltip("The CableTensionPlanner instance for physics calculations.")]
    public CableTensionPlanner tensionPlanner;

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

    public BaseController<Wrench> controller;

    // Static frame reference captured only at startup to prevent drift
    private TrackerData robot_frame_tracker;
    // The vector representing the direction of gravity in the world frame.
    private static double3 gravity_vec = 9.81f * new double3(0, 0, -1);

    private void Start()
    {
        if (!ValidateModules())
        {
            enabled = false; // Disable this script if modules are missing
            return;
        }

        if (!trackerManager.Initialize())
        {
            Debug.LogError("Failed to initialize TrackerManager.", this);
            enabled = false;
            return;
        }
        if (!tensionPlanner.Initialize())
        {
            Debug.LogError("Failed to initialize CableTensionPlanner.", this);
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
        System.Threading.Thread.Sleep(100); // Brief pause for tracking stability
        trackerManager.GetFrameTrackerData(out robot_frame_tracker);

        // Initialize visualizer now that frame pose is available
        ReadOnlySpan<Vector3> pulleyPositions = tensionPlanner.GetPulleyPositionsInRobotFrame();
        if (!visualizer.Initialize(robot_frame_tracker.PoseMatrix, pulleyPositions))
        {
            Debug.LogError("Failed to initialize RobotVisualizer.", this);
            enabled = false;
            return;
        }

        // Start the TCP connection if enabled
        if (isLabviewControlEnabled)
        {
            tcpCommunicator.ConnectToServer();
        }

        // Initialize Controller Here
        // Timestep resolution = 0.05 second, MPC prediction horizon = 10 timesteps
        // controller = new MPCController(0.05, 10);
        /* We need to initialize the controller here */
    }

    private void Update()
    {
        // 1. Get the latest raw tracker data (in the arbitrary, RIGHT-HANDED OpenVR/Vive coordinate system).
        TrackerData rawComData, rawEndEffectorData;
        trackerManager.GetCoMTrackerData(out rawComData);
        trackerManager.GetEndEffectorTrackerData(out rawEndEffectorData);

        // 2. Update visuals (delegated; safe because startup validation guarantees references)
        visualizer.UpdateTrackerVisuals(rawComData, rawEndEffectorData, robot_frame_tracker);

        // Call GetEEPositionRelativeToFrame and print its position
        Matrix4x4 eePose_robotFrame = GetEEPoseRelativeToFrame();
        //Debug.Log($"End Effector Pose Relative to Frame:\n" +
        //     $"[{eePose_robotFrame.m00:F4}, {eePose_robotFrame.m01:F4}, {eePose_robotFrame.m02:F4}, {eePose_robotFrame.m03:F4}]\n" +
        //     $"[{eePose_robotFrame.m10:F4}, {eePose_robotFrame.m11:F4}, {eePose_robotFrame.m12:F4}, {eePose_robotFrame.m13:F4}]\n" +
        //     $"[{eePose_robotFrame.m20:F4}, {eePose_robotFrame.m21:F4}, {eePose_robotFrame.m22:F4}, {eePose_robotFrame.m23:F4}]\n" +
        //     $"[{eePose_robotFrame.m30:F4}, {eePose_robotFrame.m31:F4}, {eePose_robotFrame.m32:F4}, {eePose_robotFrame.m33:F4}]");


        //Here we parse the control effort that we obtained from the controller, to be implemented
        Wrench controllerOutput = new Wrench { Force = double3.zero, Torque = double3.zero };

        // Remember: 'solverResult' points to an array managed inside CableTensionPlanner
        double[] solverResult = tensionPlanner.CalculateTensions(
            rawEndEffectorData.PoseMatrix,
            controllerOutput,
            robot_frame_tracker.PoseMatrix
        );

        Span<double> motor_command = stackalloc double[14];
        MapTensionsToMotors(solverResult, motor_command);
        // Send the calculated tensions to LabVIEW
        tcpCommunicator.SetClosedLoopControl();
        tcpCommunicator.UpdateTensionSetpoint(motor_command);
    }

    /// <summary>
    /// Helper method to get chest tracker's position relative to the static frame reference.
    /// Useful for manually recording pulley positions during calibration.
    /// </summary>
    /// <returns>Position relative to frame tracker, or Vector3.zero if not found</returns>
    public Matrix4x4 GetEEPoseRelativeToFrame()
    {
        TrackerData ee_data = trackerManager.GetEndEffectorTrackerData();
        Matrix4x4 eePose = ee_data.PoseMatrix;
        Matrix4x4 framePose_inverse = robot_frame_tracker.PoseMatrix.inverse;
        Matrix4x4 relativePose = framePose_inverse * eePose;
        return relativePose;
    }

    // Validates that all required module references are assigned in the Inspector.
    // returns True if all modules are assigned, false otherwise.
    private bool ValidateModules()
    {
        bool allValid = true;
        if (trackerManager == null) { Debug.LogError("Module not assigned in Inspector: trackerManager", this); allValid = false; }
        if (tensionPlanner == null) { Debug.LogError("Module not assigned in Inspector: tensionPlanner", this); allValid = false; }
        if (forcePlateManager == null) { Debug.LogError("Module not assigned in Inspector: forcePlateManager", this); allValid = false; }
        if (tcpCommunicator == null) { Debug.LogError("Module not assigned in Inspector: tcpCommunicator", this); allValid = false; }
        if (visualizer == null) { Debug.LogError("Visualizer not assigned in Inspector: visualizer", this); allValid = false; }

        return allValid;
    }

    private void OnDestroy()
    {
        // Clean shutdown of threaded components.
        // TrackerManager handles its own shutdown via its OnDestroy method.
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
