using UnityEngine;

/// <summary>
/// Handles all visualization elements for the robot system that already exist:
/// tracker visuals (CoM, End-Effector, Frame) and the visualization camera.
/// Converts from RH robot frame (OpenVR) to Unity's LH frame for rendering.
/// </summary>
public class RobotVisualizer : MonoBehaviour
{
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
    public bool Initialize(Matrix4x4 framePoseMatrix, Vector3[] pulleyPositionsRobotFrame)
    {
        // Validate required references
        bool ok = true;
        if (comTrackerVisual == null) { Debug.LogError("RobotVisualizer: comTrackerVisual not assigned", this); ok = false; }
        if (endEffectorVisual == null) { Debug.LogError("RobotVisualizer: endEffectorVisual not assigned", this); ok = false; }
        if (frameTrackerVisual == null) { Debug.LogError("RobotVisualizer: frameTrackerVisual not assigned", this); ok = false; }
        if (visualizationCamera == null) { Debug.LogError("RobotVisualizer: visualizationCamera not assigned", this); ok = false; }
        if (!ok) return false;

        // Initialize visual scales once for handedness conversion (LH prefab design to RH coordinate system)
        comTrackerVisual.localScale = new Vector3(1, 1, -1);
        endEffectorVisual.localScale = new Vector3(1, 1, -1);
        frameTrackerVisual.localScale = new Vector3(1, 1, -1);

        // Update the frame tracker visual using the static frame
        UpdateVisual(frameTrackerVisual, framePoseMatrix);
        UpdateVisualizationCamera(framePoseMatrix);
        visualizationCamera.nearClipPlane = 0.001f;

        // Create pulley spheres once at startup using Unity primitives
        if (pulleyPositionsRobotFrame != null && pulleyPositionsRobotFrame.Length > 0)
        {
            for (int i = 0; i < pulleyPositionsRobotFrame.Length; i++)
            {
                // Compute world position in RH frame: frame * local
                Vector3 worldRH = framePoseMatrix.MultiplyPoint3x4(pulleyPositionsRobotFrame[i]);
                // Use UpdateVisual to convert RH->LH by constructing a pose with translation only
                Matrix4x4 pulleyPoseRH = Matrix4x4.TRS(worldRH, Quaternion.identity, Vector3.one);

                var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.name = $"Pulley_{i + 1}";
                sphere.transform.SetParent(transform, worldPositionStays: false);
                sphere.transform.localScale = Vector3.one * pulleySphereSize;
                // Place in Unity world via UpdateVisual (handles mirror)
                UpdateVisual(sphere.transform, pulleyPoseRH);
                // Remove collider to avoid interactions
                var col = sphere.GetComponent<Collider>();
                if (col != null) Destroy(col);
            }
        }

        return true;
    }

    /// <summary>
    /// Update the three tracker visuals using current (RH) pose matrices.
    /// </summary>
    public void UpdateTrackerVisuals(TrackerData comData, TrackerData endEffectorData, TrackerData frameData)
    {
        UpdateVisual(comTrackerVisual, comData.PoseMatrix);
        UpdateVisual(endEffectorVisual, endEffectorData.PoseMatrix);
        UpdateVisual(frameTrackerVisual, frameData.PoseMatrix);
    }

    /// <summary>
    /// Updates the visualization camera to position it relative to the robot frame
    /// and look at the frame tracker visual.
    /// </summary>
    public void UpdateVisualizationCamera(Matrix4x4 framePoseMatrix)
    {
        Vector3 worldCameraPos = framePoseMatrix.MultiplyPoint3x4(cam_pos_robotFrame);
        visualizationCamera.transform.position = new Vector3(worldCameraPos.x, worldCameraPos.y, -worldCameraPos.z);
        visualizationCamera.transform.LookAt(frameTrackerVisual.position, -frameTrackerVisual.forward);
        
        Vector3 vectorDistance = visualizationCamera.transform.position - frameTrackerVisual.position;
        Debug.Log($"Vectorial distance from VisualizationCamera to FrameTrackerVisual: {vectorDistance}");
    }

    /// <summary>
    /// Applies RH->LH conversion and updates a visual Transform from a pose matrix.
    /// Robot coordinate system: X=right, Y=forward, Z=up
    /// Unity coordinate system: X=right, Y=up, Z=forward
    /// </summary>
    private static void UpdateVisual(Transform visual, Matrix4x4 poseMatrix)
    {
        Matrix4x4 unityMatrix = mirror_matrix * poseMatrix * mirror_matrix;
        visual.position = unityMatrix.GetColumn(3);
        visual.rotation = Quaternion.LookRotation(unityMatrix.GetColumn(2), unityMatrix.GetColumn(1));
    }
}
