using UnityEngine;
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

    public BaseController controller;

    // Static frame reference captured only at startup to prevent drift
    private readonly TrackerData robot_frame_tracker = new TrackerData();
    // The vector representing the direction of gravity in the world frame.
    private static Vector3 gravity_vec = 9.81f * new Vector3(0, 0, -1);

    private void Start()
    {
        SetProcessCoresAndPriority();
        if (!ValidateModules())
        {
            enabled = false; // Disable this script if modules are missing
            return;
        }

        // Initialize all modules. Since we are using a pure Vive-based coordinate system,
        // there is no complex calibration sequence needed at startup.
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
        if (!tcpCommunicator.Initialize(tensionPlanner.matrixCols))
        {
            Debug.LogError("Failed to initialize LabviewTcpCommunicator.", this);
            enabled = false;
            return;
        }

        // Capture static frame reference to prevent drift during operation
        System.Threading.Thread.Sleep(100); // Brief pause for tracking stability
        robot_frame_tracker.PoseMatrix = trackerManager.GetFrameTrackerData().PoseMatrix;

        // Initialize visualizer now that frame pose is available
        Vector3[] pulleyPositions = tensionPlanner.GetPulleyPositionsInRobotFrame();
        if (!visualizer.Initialize(robot_frame_tracker.PoseMatrix, pulleyPositions))
        {
            Debug.LogError("Failed to initialize RobotVisualizer.", this);
            enabled = false;
            return;
        }

        Debug.Log("All robot modules initialized successfully.");

        // Start the TCP connection if enabled
        if (isLabviewControlEnabled)
        {
            tcpCommunicator.ConnectToServer();
        }

        
        // Initialize Controller Here
        // Timestep resolution = 0.05 second, MPC prediction horizon = 10 timesteps
        controller = new MPCController(0.05, 10);
        
        /* We need to initialize the controller here */
    }

    private void Update()
    {
        // 1. Get the latest raw tracker data (in the arbitrary, RIGHT-HANDED OpenVR/Vive coordinate system).
        TrackerData rawComData = trackerManager.GetCoMTrackerData();
        TrackerData rawEndEffectorData = trackerManager.GetEndEffectorTrackerData();

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
        ControllerOutput output = null;
        
        // Test call to CableTensionPlanner.CalculateTensions
        double[] tensions = tensionPlanner.CalculateTensions(
            rawEndEffectorData.PoseMatrix,
            output.comForce,
            output.comTorque,
            robot_frame_tracker.PoseMatrix
        );
        // Send the calculated tensions to LabVIEW
        tcpCommunicator.UpdateTensionSetpoint(tensions);
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

    private void SetProcessCoresAndPriority()
    {
        using (System.Diagnostics.Process process = System.Diagnostics.Process.GetCurrentProcess())
        {
            // Set to high priority but not realtime to avoid system lockups
            process.PriorityClass = System.Diagnostics.ProcessPriorityClass.High;

            // Restrict to first 8 cores (P-cores on i9-13900K)
            // 0xFF = 255 = 11111111 in binary
            process.ProcessorAffinity = new IntPtr(0xFF);
            Debug.Log($"Process priority set to {process.PriorityClass}, running on cores 0-7");
        }
    }
}
