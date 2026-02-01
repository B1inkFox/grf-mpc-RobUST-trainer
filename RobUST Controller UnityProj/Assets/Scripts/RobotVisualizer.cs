using UnityEngine;
using System;
/// <summary>
/// Handles all visualization elements for the robot system that already exist:
/// tracker visuals (CoM, End-Effector, Frame) and the visualization camera.
/// Converts from RH robot frame (OpenVR) to Unity's LH frame for rendering.
/// 
/// Thread-safe: External code can call SetTrackerData() from any thread.
/// Actual transform updates happen in Update() on the main thread.
/// </summary>
public class RobotVisualizer : MonoBehaviour
{
    [Tooltip("Enable or disable the robot visualization.")]
    public Boolean isActive = true;

    [Header("Tracker Visuals")]
    [Tooltip("A visual representation of the CoM tracker.")]
    public Transform comTrackerVisual;

    [Tooltip("A visual representation of the end-effector tracker.")]
    public Transform endEffectorVisual;

    [Tooltip("A visual representation of the frame tracker (world origin).")]
    public Transform frameTrackerVisual;

    [Header("Visualization Camera")]
    [Tooltip("Camera used for robot visualization")]
    public Camera visualizationCamera;

    [Tooltip("Camera position expressed in robot frame coordinates (Z-up)")]
    public Vector3 cam_pos_robotFrame;

    // Pulley spheres: simple, init-only visualization
    private float pulleySphereSize = 0.2f;
    private RobUSTDescription robot;

    // Thread-safe cached tracker data (written from control thread, read in Update)
    private readonly object dataLock = new object();
    private TrackerData cachedComData;
    private TrackerData cachedEndEffectorData;
    private TrackerData cachedFrameData;
    private volatile bool hasNewData = false;
    private bool isInitialized = false;

    // RH (robot/OpenVR) -> Unity (LH) mirror matrix
    private static readonly Matrix4x4 mirror_matrix = new Matrix4x4(
        new Vector4(1, 0, 0, 0),
        new Vector4(0, 1, 0, 0),
        new Vector4(0, 0, -1, 0),
        new Vector4(0, 0, 0, 1)
    );

    /// <summary>
    /// Initialize visual scales, set the static frame visual, and update the camera.
    /// Returns true if successful. Perform all validation here to avoid runtime checks.
    /// Call this after RobotController captures the static frame matrix.
    /// </summary>
    /// <param name="framePoseMatrix">Static frame pose in robot (RH) coordinates.</param>
    /// <param name="pulleyPositionsRobotFrame">Pulley points relative to the frame tracker, in RH robot frame.</param>
    public bool Initialize(Matrix4x4 framePoseMatrix, RobUSTDescription robotDescription)
    {
        robot = robotDescription;

        // Validate required references
        bool ok = true;
        if (comTrackerVisual == null) { Debug.LogError("RobotVisualizer: comTrackerVisual not assigned", this); ok = false; }
        if (endEffectorVisual == null) { Debug.LogError("RobotVisualizer: endEffectorVisual not assigned", this); ok = false; }
        if (frameTrackerVisual == null) { Debug.LogError("RobotVisualizer: frameTrackerVisual not assigned", this); ok = false; }
        if (visualizationCamera == null) { Debug.LogError("RobotVisualizer: visualizationCamera not assigned", this); ok = false; }
        if (!ok) return false;

        // Initialize visual scales once for handedness conversion (LH prefab design to RH coordinate system)
        comTrackerVisual.localScale = .2f * new Vector3(1, 1, -1);
        endEffectorVisual.localScale = .2f * new Vector3(1, 1, -1);
        frameTrackerVisual.localScale = .2f * new Vector3(1, 1, -1);

        ApplyVisual(frameTrackerVisual, framePoseMatrix);
        UpdateVisualizationCamera();

        // Create pulley spheres once at startup using Unity primitives
        for (int i = 0; i < robot.FramePulleyPositions.Length; i++)
        {
            Vector3 pulleyPosLocal = new Vector3(
                (float)robot.FramePulleyPositions[i].x,
                (float)robot.FramePulleyPositions[i].y,
                (float)robot.FramePulleyPositions[i].z
            );

            // Compute world position in RH frame: frame * local
            Vector3 worldRH = framePoseMatrix.MultiplyPoint3x4(pulleyPosLocal);
            // Use ApplyVisual to convert RH->LH by constructing a pose with translation only
            Matrix4x4 pulleyPose = Matrix4x4.TRS(worldRH, Quaternion.identity, Vector3.one);

            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"Pulley_{i + 1}";
            sphere.transform.localScale = Vector3.one * pulleySphereSize;
            ApplyVisual(sphere.transform, pulleyPose); // Place in Unity world via ApplyVisual (handles mirror)
            
            // Remove collider to avoid interactions
            var col = sphere.GetComponent<Collider>();
            if (col != null) Destroy(col);
        }

        isInitialized = true;
        return true;
    }

    /// <summary>
    /// Thread-safe: Cache new tracker data to be applied on next Update().
    /// Call this from control thread.
    /// </summary>
    public void SetTrackerData(TrackerData comData, TrackerData endEffectorData, TrackerData frameData)
    {
        lock (dataLock)
        {
            cachedComData = comData;
            cachedEndEffectorData = endEffectorData;
            cachedFrameData = frameData;
            hasNewData = true;
        }
    }

    /// <summary>
    /// Main thread: Apply cached tracker data to Unity transforms.
    /// </summary>
    private void Update()
    {
        if (!isInitialized || !hasNewData || !isActive) return;

        TrackerData com, ee, frame;
        lock (dataLock)
        {
            com = cachedComData;
            ee = cachedEndEffectorData;
            frame = cachedFrameData;
            hasNewData = false;
        }
    
        ApplyVisual(comTrackerVisual, com.PoseMatrix);
        ApplyVisual(endEffectorVisual, ee.PoseMatrix);
        ApplyVisual(frameTrackerVisual, frame.PoseMatrix);
    }

    /// <summary>
    /// Updates the visualization camera to position it relative to the robot frame
    /// and look at the frame tracker visual. Assumes frame tracker visual is already up to date.
    /// </summary>
    public void UpdateVisualizationCamera()
    {
        Matrix4x4 frameVisualPose = frameTrackerVisual.transform.localToWorldMatrix;
        Vector3 cam_pos_unity = frameVisualPose.MultiplyPoint3x4(cam_pos_robotFrame);

        visualizationCamera.transform.position = cam_pos_unity;
        visualizationCamera.transform.LookAt(frameTrackerVisual.position, -frameTrackerVisual.forward);
        // Pan the camera 10 degrees to the right (relative to its current view direction)
        visualizationCamera.transform.Rotate(0f, 10f, 0f, Space.Self);

        float distance = Vector3.Distance(visualizationCamera.transform.position, frameTrackerVisual.position);
    }

    /// <summary>
    /// Applies RH->LH conversion and updates a visual Transform from a pose matrix.
    /// Robot coordinate system: X=right, Y=forward, Z=up
    /// Unity coordinate system: X=right, Y=up, Z=forward
    /// Must be called from main thread.
    /// </summary>
    private static void ApplyVisual(Transform visual, Matrix4x4 raw_poseMatrix)
    {
        Matrix4x4 unityMatrix = mirror_matrix * raw_poseMatrix * mirror_matrix;
        visual.position = unityMatrix.GetColumn(3);
        visual.rotation = Quaternion.LookRotation(unityMatrix.GetColumn(2), unityMatrix.GetColumn(1));
    }
}
