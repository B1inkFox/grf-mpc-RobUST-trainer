using UnityEngine;
using System;

/// <summary>
/// Performs the physics calculations to determine the required cable tensions
/// based on the robot's state and external forces.
/// </summary>
public class CableTensionPlanner : MonoBehaviour
{
    [Tooltip("Number of columns (cables) in the structure matrix.")]
    public int matrixCols = 4;

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
    private double[] tensions;
    private Vector3[] localAttachmentPoints; // r_i vectors, local to the end-effector
    private Vector3[] framePulleyPositions;  // Local positions of the pulleys relative to the frame tracker

    // Pre-allocated QP matrices (allocated once, reused every calculation)
    private double[,] quadratic;
    private double[] linear;
    private double[,] sMatrix; // 6x4 structure matrix for force/torque constraints
    private double[] tensionLower; // Box constraint lower bounds
    private double[] tensionUpper; // Box constraint upper bounds
    private double[] lowerWrenchBounds;
    private double[] upperWrenchBounds;
    private double[] scale;
    
    private double forceEpsilon = 0.01;
    private double torqueEpsilon = 1000;
    private double minTension = 10.0;
    private double maxTension = 200;

    // Add new fields for QP state and warm start solution
    private alglib.minqpstate qpState;
    private double[] previousSolution;

    /// <summary>
    /// Initializes the tension planner by pre-calculating constant geometric properties.
    /// Called by RobotController in the correct dependency order.
    /// </summary>
    /// <returns>True if initialization succeeded, false otherwise</returns>
    public bool Initialize()
    {
        // Validate configuration
        if (chest_AP_distance <= 0 || chest_ML_distance <= 0)
        {
            Debug.LogError("Chest dimensions must be positive values.", this);
            return false;
        }

        // Pre-allocate all data structures
        tensions = new double[matrixCols];
        localAttachmentPoints = new Vector3[matrixCols];
        framePulleyPositions = new Vector3[matrixCols];

        // Pre-allocate QP structures
        quadratic = new double[matrixCols, matrixCols];
        linear = new double[matrixCols];
        sMatrix = new double[6, matrixCols]; // 6xnumCables structure matrix
        tensionLower = new double[matrixCols]; // box constraints arrays
        tensionUpper = new double[matrixCols];
        lowerWrenchBounds = new double[6]; // linear constraints arrays
        upperWrenchBounds = new double[6];

        // scale array (required for Dense IPM)
        scale = new double[matrixCols];

        // Initialize quadratic matrix: P = 2*Identity (constant, never changes)
        for (int i = 0; i < matrixCols; i++)
            quadratic[i, i] = 2.0;
            
        // Initialize tension bounds (constant, never changes)
        for (int i = 0; i < matrixCols; i++)
        {
            tensionLower[i] = minTension;  // Minimum tension
            tensionUpper[i] = maxTension; // Maximum tension
        }
        // Initialize scale array (required for Dense IPM - use typical tension magnitude)
        for (int i = 0; i < matrixCols; i++)
            scale[i] = 50.0; // Mid-range tension value for scaling

        // Initialize previousSolution with a reasonable default
        previousSolution = new double[matrixCols];
        for (int i = 0; i < matrixCols; i++)
            previousSolution[i] = 50.0; // default tension value
        
        // initialize qp solver
        alglib.minqpcreate(matrixCols, out qpState); // Initialize QP state once and reuse it for subsequent solves
        alglib.minqpsetalgodenseipm(qpState, 1e-6);  // Stopping tolerance: 1e-6
        alglib.minqpsetquadraticterm(qpState, quadratic, true);
        alglib.minqpsetlinearterm(qpState, linear);
        alglib.minqpsetbc(qpState, tensionLower, tensionUpper);

        // Pre-compute fixed pulley positions in the RIGHT-HANDED coordinate system, relative to robot frame tracker 
        framePulleyPositions[0] = new Vector3(-.5f, 1.0f, 1.0f);   // Front-Right Top (Motor index 0)
        framePulleyPositions[1] = new Vector3(-.5f, 0.0f, 1.0f);  // Front-Left Top (Motor index 1)
        framePulleyPositions[2] = new Vector3(.5f, 0.0f, 1.0f); // Back-Left Top (Motor index 2)
        framePulleyPositions[3] = new Vector3(.5f, 1.0f, 1.0f);  // Back-Right Top (Motor index 3)
        framePulleyPositions[4] = new Vector3(-.5f, 1.0f, -1.0f);   // Front-Right Bottom (Motor index 4)
        framePulleyPositions[5] = new Vector3(-.5f, 0.0f, -1.0f);  // Front-Left Bottom (Motor index 5)
        framePulleyPositions[6] = new Vector3(.5f, 0.0f, -1.0f); // Back-Left Bottom (Motor index 6)
        framePulleyPositions[7] = new Vector3(.5f, 1.0f, -1.0f);  // Back-Right Bottom (Motor index 7)

        // Pre-compute local attachment points based on belt geometry in the RIGHT-HANDED coordinate system
        float halfAP = chest_AP_distance / 2.0f;
        float halfML = chest_ML_distance / 2.0f;
        float ap_factor = (beltSize == BeltSize.Small) ? 0.7f : 0.85f;
        float ml_factor = (beltSize == BeltSize.Small) ? 0.8f : 0.95f;

        localAttachmentPoints[0] = new Vector3(-halfML * ml_factor, -halfAP * ap_factor, 0);  // Front-Right (Motor index 0)
        localAttachmentPoints[1] = new Vector3(halfML * ml_factor, -halfAP * ap_factor, 0); // Front-Left (Motor index 1)
        localAttachmentPoints[2] = new Vector3(halfML * ml_factor, 0, 0);// Back-Left (Motor index 2)
        localAttachmentPoints[3] = new Vector3(-halfML * ml_factor, 0, 0); // Back-Right (Motor index 3)
        // repeat for bottom cables
        localAttachmentPoints[4] = new Vector3(-halfML * ml_factor, -halfAP * ap_factor, 0);  // Front-Right (Motor index 4)
        localAttachmentPoints[5] = new Vector3(halfML * ml_factor, -halfAP * ap_factor, 0); // Front-Left (Motor index 5)
        localAttachmentPoints[6] = new Vector3(halfML * ml_factor, 0, 0);// Back-Left (Motor index 6)
        localAttachmentPoints[7] = new Vector3(-halfML * ml_factor, 0, 0); // Back-Right (Motor index 7)

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
    public double[] CalculateTensions(Matrix4x4 endEffectorPose, Vector3 desiredForce, Vector3 desiredTorque, Matrix4x4 robotFramePose)
    {
        // Build Structure Matrix 
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
            Vector3 r_i_eeFrame = localAttachmentPoints[i] - new Vector3(0.0f, -chest_AP_distance / 2.0f, 0.0f);
            Vector3 r_i_robotFrame = EE_to_RobotFrame.MultiplyPoint3x4(r_i_eeFrame);
            Vector3 torqueComponent = Vector3.Cross(r_i_robotFrame, u_i);

            // 6. Fill directly into sMatrix
            sMatrix[0, i] = u_i.x;
            sMatrix[1, i] = u_i.y;
            sMatrix[2, i] = u_i.z;
            sMatrix[3, i] = torqueComponent.x;
            sMatrix[4, i] = torqueComponent.y;
            sMatrix[5, i] = torqueComponent.z;
        }

        // --- 2. Update Constraint Values (2-sided with epsilon tolerance) ---
        lowerWrenchBounds[0] = desiredForce.x - forceEpsilon;
        upperWrenchBounds[0] = desiredForce.x + forceEpsilon;
        lowerWrenchBounds[1] = desiredForce.y - forceEpsilon;
        upperWrenchBounds[1] = desiredForce.y + forceEpsilon;
        lowerWrenchBounds[2] = desiredForce.z - forceEpsilon;
        upperWrenchBounds[2] = desiredForce.z + forceEpsilon;
        lowerWrenchBounds[3] = desiredTorque.x - torqueEpsilon;
        upperWrenchBounds[3] = desiredTorque.x + torqueEpsilon;
        lowerWrenchBounds[4] = desiredTorque.y - torqueEpsilon;
        upperWrenchBounds[4] = desiredTorque.y + torqueEpsilon;
        lowerWrenchBounds[5] = desiredTorque.z - torqueEpsilon;
        upperWrenchBounds[5] = desiredTorque.z + torqueEpsilon;

        // --- 3. Solve QP Problem using Dense IPM ---
        alglib.minqpsetlc2dense(qpState, sMatrix, lowerWrenchBounds, upperWrenchBounds, 6);
        alglib.minqpsetstartingpoint(qpState, previousSolution); // warm start
        
        // Optimize
        alglib.minqpoptimize(qpState);
        alglib.minqpresults(qpState, out tensions, out var report);

        // Update the previous solution for the next call
        Array.Copy(tensions, previousSolution, matrixCols);

        // --- 4. Check Results ---
        if (report.terminationtype < 0)
        {
            Debug.LogWarning($"QP solver failed: {report.terminationtype}");
            return new double[matrixCols];
        }

        Debug.Log("Computed tensions: [" + string.Join(", ", System.Array.ConvertAll(tensions, t => t.ToString("F3"))) + "]");

        // // Calculate the actual wrench (forces and torques) from the computed tensions
        // double[] actualWrench = new double[6];
        // for (int i = 0; i < 6; i++)
        // {
        //     actualWrench[i] = 0;
        //     for (int j = 0; j < matrixCols; j++)
        //     {
        //         actualWrench[i] += sMatrix[i, j] * tensions[j];
        //     }
        // }
        // // Print the actual wrench
        // Debug.Log("Actual wrench [Fx, Fy, Fz, Tx, Ty, Tz]: [" + 
        //           string.Join(", ", System.Array.ConvertAll(actualWrench, w => w.ToString("F3"))) + "]");
                  
        return tensions;
    }

    /// <summary>
    /// Returns a copy of the fixed pulley positions in the RIGHT-HANDED robot frame
    /// (positions are relative to the frame tracker pose).
    /// </summary>
    public Vector3[] GetPulleyPositionsInRobotFrame()
    {
        var copy = new Vector3[framePulleyPositions.Length];
        Array.Copy(framePulleyPositions, copy, framePulleyPositions.Length);
        return copy;
    }
}
