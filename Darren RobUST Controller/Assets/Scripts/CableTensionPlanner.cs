using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Performs the physics calculations to determine the required cable tensions
/// based on the robot's state and external forces.
/// </summary>
public class CableTensionPlanner : MonoBehaviour
{
    [Tooltip("Number of rows in the structure matrix.")]
    public int matrixRows = 6;

    [Tooltip("Number of columns (cables) in the structure matrix.")]
    public int matrixCols = 4;

    [Header("Motor Mapping")]
    [Tooltip("The motor number corresponding to each column of the structure matrix. Size must match 'matrixCols'.")]
    public int[] motorNumbers;

    [Header("Chest Anterior-Posterior Distance")]
    [Tooltip("Measured Thickness of the chest in the anterior-posterior direction.")]
    public float chest_AP_distance = 0;

    [Header("Chest Medial-Lateral Distance")]
    [Tooltip("Measured width of the chest in the medial-lateral direction.")]
    public float chest_ML_distance = 0;

    public enum BeltSize { Small, Large }
    [Header("Belt Size")]
    [Tooltip("Select the belt size for the user.")]
    public BeltSize beltSize = BeltSize.Small;


    // Pre-computed constants
    private Matrix<float> sMatrix;
    private float[] tensions;
    private Vector3[] localAttachmentPoints; // r_i vectors, local to the end-effector
    private Vector3[] framePulleyPositions;  // Local positions of the pulleys relative to the frame tracker

    /// <summary>
    /// Initializes the tension planner by pre-calculating constant geometric properties.
    /// Called by RobotController in the correct dependency order.
    /// </summary>
    /// <returns>True if initialization succeeded, false otherwise</returns>
    public bool Initialize()
    {
        // Validate configuration
        if (motorNumbers == null || motorNumbers.Length != matrixCols)
        {
            Debug.LogError($"Motor numbers array must have {matrixCols} elements to match matrix columns.", this);
            return false;
        }

        if (chest_AP_distance <= 0 || chest_ML_distance <= 0)
        {
            Debug.LogError("Chest dimensions must be positive values.", this);
            return false;
        }

        // Pre-allocate all data structures
        tensions = new float[matrixCols];
        sMatrix = Matrix<float>.Build.Dense(matrixRows, matrixCols);
        localAttachmentPoints = new Vector3[matrixCols];
        framePulleyPositions = new Vector3[matrixCols];

        // Pre-compute fixed pulley positions in the RIGHT-HANDED coordinate system,
        // assuming the frame tracker is the origin.
        framePulleyPositions[0] = new Vector3(0.4826f, 0.9652f, 0.5f);   // Front-Right (Motor 1)
        framePulleyPositions[1] = new Vector3(-0.4826f, 0.0f, 0.5f);  // Front-Left (Motor 2)
        framePulleyPositions[2] = new Vector3(-0.4826f, 0.0f, 0.5f); // Back-Left (Motor 3)
        framePulleyPositions[3] = new Vector3(0.4826f, 0.9652f, 0.5f);  // Back-Right (Motor 4)

        // Pre-compute local attachment points based on belt geometry in the RIGHT-HANDED coordinate system
        float halfAP = chest_AP_distance / 2.0f;
        float halfML = chest_ML_distance / 2.0f;
        float ap_factor = (beltSize == BeltSize.Small) ? 0.7f : 0.85f;
        float ml_factor = (beltSize == BeltSize.Small) ? 0.8f : 0.95f;

        // Note: Assuming Z is forward, X is right, Y is up (standard right-handed system)
        localAttachmentPoints[0] = new Vector3(halfML * ml_factor, 0, halfAP * ap_factor);  // Front-Right
        localAttachmentPoints[1] = new Vector3(-halfML * ml_factor, 0, halfAP * ap_factor); // Front-Left
        localAttachmentPoints[2] = new Vector3(-halfML * ml_factor, 0, -halfAP * ap_factor);// Back-Left
        localAttachmentPoints[3] = new Vector3(halfML * ml_factor, 0, -halfAP * ap_factor); // Back-Right


        Debug.Log($"CableTensionPlanner initialized for {matrixCols} cables with {beltSize} belt size.");
        return true;
    }

    /// <summary>
    /// Calculates the desired cable tensions based on real-time tracker and force data.
    /// All calculations are performed in the RIGHT-HANDED coordinate system.
    /// </summary>
    /// <param name="endEffectorPose">The 4x4 pose matrix of the end-effector in the world coordinate system.</param>
    /// <param name="desiredForce">The desired force to be applied by the cables.</param>
    /// <param name="desiredTorque">The desired torque to be applied by the cables.</param>
    /// <param name="robotFramePose">The transformation matrix of the frame-mounted vive tracker</param>
    public float[] CalculateTensions(Matrix4x4 endEffectorPose, Vector3 desiredForce, Vector3 desiredTorque, Matrix4x4 robotFramePose)
    {
        // --- Build the Structure Matrix (S) ---
        // The S matrix maps the cable tensions to the wrench (force and torque) applied to the end-effector.
        // Each column 'i' of S corresponds to a cable and is composed of:
        // - The unit vector u_i (direction of the cable)
        // - The cross product of r_i and u_i (torque generated by the cable)
        // where r_i is the vector from the end-effector's origin to the cable's attachment point.

        for (int i = 0; i < matrixCols; i++)
        {
            // 1. compute robot frame inverse HTM
            Matrix4x4 robotFrameInverse = robotFramePose.inverse;
            Matrix4x4 EE_to_RobotFrame = robotFrameInverse * endEffectorPose;

            // 2. Transform the local attachment point to its position in the robot frame coordinate system.
            Vector3 ee_point_robotFrame = EE_to_RobotFrame.MultiplyPoint3x4(localAttachmentPoints[i]);
            // framePulleyPositions[i] is already in the robot frame

            // 3. Calculate the vector for the cable, from the attachment point to the fixed pulley.
            Vector3 cableVector = framePulleyPositions[i] - ee_point_robotFrame;

            // 4. Normalize this vector to get the unit direction vector (u_i) for the force.
            Vector3 u_i = cableVector.normalized;

            // 5. Calculate the torque component for this cable (r_i x u_i).
            Vector3 r_i_eeFrame = localAttachmentPoints[i] - new Vector3(0.0f, 0.0f, chest_AP_distance / 2.0f);
            Vector3 r_i_robotFrame = EE_to_RobotFrame.MultiplyPoint3x4(r_i_eeFrame);
            Vector3 torqueComponent = Vector3.Cross(r_i_robotFrame, u_i);

            // 6. Populate the i-th column of the structure matrix S.
            // The top three rows are the force components (the unit vector u_i).
            sMatrix[0, i] = u_i.x;
            sMatrix[1, i] = u_i.y;
            sMatrix[2, i] = u_i.z;
            // The bottom three rows are the torque components.
            sMatrix[3, i] = torqueComponent.x;
            sMatrix[4, i] = torqueComponent.y;
            sMatrix[5, i] = torqueComponent.z;
        }

        // --- Solve for Tensions (Future Implementation) ---
        // At this point, the matrix S is fully constructed for the current robot pose.
        // The next step is to set up and solve a Quadratic Programming (QP) problem
        // to find the optimal tensions 't' that satisfy S*t = w, where w is the desired wrench.
        // This typically involves minimizing ||S*t - w||^2 subject to t >= 0.

        // For now, we will continue to return a zeroed array.
        for (int i = 0; i < tensions.Length; i++)
        {
            tensions[i] = 0f;
        }
        return tensions;
    }

    /// <summary>
    /// Returns a copy of the fixed pulley positions in the RIGHT-HANDED robot frame
    /// (positions are relative to the frame tracker pose).
    /// </summary>
    public Vector3[] GetPulleyPositionsInRobotFrame()
    {
        var copy = new Vector3[framePulleyPositions.Length];
        for (int i = 0; i < framePulleyPositions.Length; i++) copy[i] = framePulleyPositions[i];
        return copy;
    }
}
