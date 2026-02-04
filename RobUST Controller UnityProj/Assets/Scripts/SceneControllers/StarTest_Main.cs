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
public class StarTest_Main : MonoBehaviour
{
    #region Inspector Assigned Modules
    [Header("Module References")]
    [Tooltip("The TrackerManager instance that provides tracker data.")]
    public TrackerManager trackerManager;
    [Tooltip("The ForcePlateManager instance for reading force plate data.")]
    public ForcePlateManager forcePlateManager;
    [Tooltip("Handles tracker/camera visuals. Keeps RobotController logic-only.")]
    public RobotVisualizer visualizer;
    #endregion

    #region User Configuration
    [Header("User Geometry Configuration")]
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
    #endregion

    #region State Definitions
    public enum STAR_STATE 
    { 
        OFF, 
        FORWARD, 
        BACKWARD, 
        LEFT, 
        RIGHT 
    }
    public volatile STAR_STATE currState = STAR_STATE.OFF;
    #endregion

    #region Flags
    private bool isForcePlateEnabled = true;
    private volatile bool isRunning = false;
    #endregion

    #region Internal Data & Frames
    private RobUSTDescription robotDescription;
    private double4x4 raw_robotFrame_pose;
    private double4x4 robotFrame_inv;
    #endregion

    private void Start()
    {
        using (System.Diagnostics.Process p = System.Diagnostics.Process.GetCurrentProcess())
        {
            p.PriorityClass = System.Diagnostics.ProcessPriorityClass.High;
        }
        robotDescription = RobUSTDescription.Create(8, chestAPDistance, chestMLDistance, userMass, userShoulderWidth, userHeight);

        if (!ValidateModules())
        {
            enabled = false; 
            return;
        }

        if (chestAPDistance <= 0 || chestMLDistance <= 0)
        {
            Debug.LogError("Chest dimensions must be positive values.", this);
            enabled = false;
            return;
        }

        // Initialize Modules
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
        if (!visualizer.Initialize(robotDescription))
        {
            Debug.LogError("Failed to initialize RobotVisualizer.", this);
            enabled = false;
            return;
        }

        System.Threading.Thread.Sleep(500); // allow tracker thread to go live
        trackerManager.GetFrameTrackerData(out TrackerData robot_frame_tracker); // import Capture static frame at startup 
        raw_robotFrame_pose = ToDouble4x4(robot_frame_tracker.PoseMatrix);
        robotFrame_inv = math.fastinverse(raw_robotFrame_pose);

        isRunning = true;
    }

    private void Update()
    {
        // get poses in RF
        trackerManager.GetCoMTrackerData(out TrackerData raw_comTrackerData);
        trackerManager.GetEndEffectorTrackerData(out TrackerData raw_eeTrackerData);
        double4x4 comPose_RF = math.mul(robotFrame_inv, ToDouble4x4(raw_comTrackerData.PoseMatrix));
        double4x4 eePose_RF = math.mul(robotFrame_inv, ToDouble4x4(raw_eeTrackerData.PoseMatrix));
        // get force plate data (already in RF)
        forcePlateManager.GetForcePlateData(0, out ForcePlateData fp0);
        forcePlateManager.GetForcePlateData(1, out ForcePlateData fp1);
        // implement star test logic

        // send to visualizer
        visualizer.PushState(comPose_RF, eePose_RF, fp0, fp1);
    }

    // Validates that all required module references are assigned in the Inspector.
    // returns True if all modules are assigned, false otherwise.
    private bool ValidateModules()
    {
        bool allValid = true;
        if (trackerManager == null) { Debug.LogError("Module not assigned in Inspector: trackerManager", this); allValid = false; }
        if (forcePlateManager == null) { Debug.LogError("Module not assigned in Inspector: forcePlateManager", this); allValid = false; }
        if (visualizer == null) { Debug.LogError("Visualizer not assigned in Inspector: visualizer", this); allValid = false; }

        return allValid;
    }

    private void OnDestroy()
    {
        // Clean shutdown of threaded components.
        // TrackerManager handles its own shutdown via its OnDestroy method.
        isRunning = false;
    }

    // Local helper
    private static double4x4 ToDouble4x4(in Matrix4x4 m)
    {
        return new double4x4(
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }

}
