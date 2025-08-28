using UnityEngine;

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


    // Static frame reference captured at startup to prevent drift
    private readonly TrackerData robot_frame_tracker = new TrackerData();
    // The vector representing the direction of gravity in the world frame.
    private static Vector3 gravity_vec = 9.81f * new Vector3(0, 0, -1);

    private void Start()
    {
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
            enabled = false;
            return;
        }
        if (!tcpCommunicator.Initialize(tensionPlanner.motorNumbers))
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
    }

    private void Update()
    {
        // 1. Get the latest raw tracker data (in the arbitrary, RIGHT-HANDED OpenVR/Vive coordinate system).
        TrackerData rawComData = trackerManager.GetCoMTrackerData();
        TrackerData rawEndEffectorData = trackerManager.GetEndEffectorTrackerData();

        // 2. Update visuals (delegated; safe because startup validation guarantees references)
        visualizer.UpdateTrackerVisuals(rawComData, rawEndEffectorData, robot_frame_tracker);

        // Call GetEEPositionRelativeToFrame and print its position
        Vector3 eeRelativePos = GetEEPositionRelativeToFrame();
        //Debug.Log($"End Effector Position Relative to Frame: ({eeRelativePos.x:F6}, {eeRelativePos.y:F6}, {eeRelativePos.z:F6})");

        // Test call to CableTensionPlanner.CalculateTensions
        float[] tensions = tensionPlanner.CalculateTensions(
            rawEndEffectorData.PoseMatrix,
            Vector3.zero, // desiredForce (zero for test)
            Vector3.zero, // desiredTorque (zero for test)
            robot_frame_tracker.PoseMatrix
        );
    }

    /// <summary>
    /// Helper method to get chest tracker's position relative to the static frame reference.
    /// Useful for manually recording pulley positions during calibration.
    /// </summary>
    /// <returns>Position relative to frame tracker, or Vector3.zero if not found</returns>
    public Vector3 GetEEPositionRelativeToFrame()
    {
        TrackerData ee_data = trackerManager.GetEndEffectorTrackerData();
        Vector3 tracker_pos = ee_data.PoseMatrix.GetColumn(3);
        Matrix4x4 framePose_inverse = robot_frame_tracker.PoseMatrix.inverse;
        Vector3 relativePos = framePose_inverse.MultiplyPoint3x4(tracker_pos);
        return relativePos;
    }

    /// <summary>
    /// This is the placeholder for your high-level control logic.
    /// It determines the force and torque to be applied by the cables.
    /// </summary>
    private (Vector3 force, Vector3 torque) CalculateDesiredWrench(Matrix4x4 comPose, Matrix4x4 endEffectorPose)
    {
        // --- FUTURE IMPLEMENTATION ---
        // This is where you would implement logic like force fields, perturbations, etc.

        // For now, return a zero wrench (the robot will do nothing).
        return (Vector3.zero, Vector3.zero);
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
}
