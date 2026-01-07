using UnityEngine;
using Unity.Mathematics;
using System;
using static Unity.Mathematics.math;

/// <summary>
/// Performs the physics calculations to determine the required cable tensions
/// based on the robot's state and external forces.
/// 
/// OPTIMIZATION NOTES:
/// - Uses Unity.Mathematics (SIMD-optimized) for all vector/matrix operations
/// - Pre-allocates all arrays during Initialize() - no runtime allocations
/// - ALGLIB still allocates internally (~2-5KB per solve) - see CableTensionPlannerNative for zero-alloc version
/// - Reuses QP state with warm-start for faster convergence
/// </summary>
public class CableTensionPlanner : MonoBehaviour
{
    [Tooltip("Number of columns (cables) in the structure matrix.")]
    public int matrixCols = 8;

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

    // Pre-computed constants - using Unity.Mathematics types for SIMD
    private double[] tensions;
    private double3[] localAttachmentPoints;  // r_i vectors, local to the end-effector (double3 for SIMD)
    private double3[] framePulleyPositions;   // Pulley positions in robot frame (double3 for SIMD)
    private Vector3[] framePulleyPositionsVec3; // Keep Vector3 version for visualizer compatibility

    // Pre-allocated QP matrices (allocated once, reused every calculation)
    private double[,] quadratic;
    private double[] linear;
    private double[,] sMatrix; // 6x8 structure matrix for force/torque constraints
    private double[] tensionLower; // Box constraint lower bounds
    private double[] tensionUpper; // Box constraint upper bounds
    private double[] lowerWrenchBounds;
    private double[] upperWrenchBounds;
    
    // Solver parameters
    private double forceEpsilon = 0.01;
    private double torqueEpsilon = 1000;
    private double minTension = 10.0;
    private double maxTension = 200;

    // Add new fields for QP state and warm start solution
    private alglib.minqpstate qpState;
    private double[] previousSolution;
    
    // center of belt end-effector in ee frame
    private double3 beltCenter_ee_frame;

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

        // Pre-allocate all data structures using Unity.Mathematics types
        tensions = new double[matrixCols];
        localAttachmentPoints = new double3[matrixCols];
        framePulleyPositions = new double3[matrixCols];
        framePulleyPositionsVec3 = new Vector3[matrixCols]; // For visualizer compatibility

        // Pre-allocate QP structures
        quadratic = new double[matrixCols, matrixCols];
        linear = new double[matrixCols];
        sMatrix = new double[6, matrixCols]; // 6xnumCables structure matrix
        tensionLower = new double[matrixCols]; // box constraints arrays
        tensionUpper = new double[matrixCols];
        lowerWrenchBounds = new double[6]; // linear constraints arrays
        upperWrenchBounds = new double[6];

        // Initialize quadratic matrix: P = 2*Identity (constant, never changes)
        for (int i = 0; i < matrixCols; i++)
            quadratic[i, i] = 2.0;
            
        // Initialize tension bounds (constant, never changes)
        for (int i = 0; i < matrixCols; i++)
        {
            tensionLower[i] = minTension;  // Minimum tension
            tensionUpper[i] = maxTension; // Maximum tension
        }

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

        // Pre-allocate fixed pulley positions relative to robot frame tracker (using double3 for SIMD)
        framePulleyPositions[0] = new double3(-0.8114, 1.6556, 0.9400);   // Front-Right Top (Motor 10)
        framePulleyPositions[1] = new double3(-0.8066, 0.0084, 0.8895);   // Front-Left Top (Motor 5)
        framePulleyPositions[2] = new double3(0.9827, 0.0592, 0.9126);    // Back-Left Top (Motor 4)
        framePulleyPositions[3] = new double3(0.9718, 1.6551, 0.9411);    // Back-Right Top (Motor 11)
        framePulleyPositions[4] = new double3(-0.8084, 1.6496, -0.3060);  // Front-Right Bottom (Motor 8)
        framePulleyPositions[5] = new double3(-0.7667, 0.0144, -0.3243);  // Front-Left Bottom (Motor 7)
        framePulleyPositions[6] = new double3(0.9748, 0.0681, -0.5438);   // Back-Left Bottom (Motor 2)
        framePulleyPositions[7] = new double3(0.9498, 1.6744, -0.5409);   // Back-Right Bottom (Motor 13)

        // Copy to Vector3 array for visualizer compatibility (init-only allocation)
        for (int i = 0; i < matrixCols; i++)
            framePulleyPositionsVec3[i] = new Vector3((float)framePulleyPositions[i].x, (float)framePulleyPositions[i].y, (float)framePulleyPositions[i].z);
            
        // Pre-allocate local attachment points based on belt geometry relative to end-effector tracker
        double halfML = chest_ML_distance / 2.0;

        localAttachmentPoints[0] = new double3(-halfML, -chest_AP_distance, 0);  // Front-Right
        localAttachmentPoints[1] = new double3(halfML, -chest_AP_distance, 0);   // Front-Left
        localAttachmentPoints[2] = new double3(halfML, 0, 0);                      // Back-Left 
        localAttachmentPoints[3] = new double3(-halfML, 0, 0);                     // Back-Right 
        // repeat for bottom cables
        localAttachmentPoints[4] = new double3(-halfML, -chest_AP_distance, 0);  // Front-Right
        localAttachmentPoints[5] = new double3(halfML, -chest_AP_distance, 0);   // Front-Left
        localAttachmentPoints[6] = new double3(halfML, 0, 0);                      // Back-Left
        localAttachmentPoints[7] = new double3(-halfML, 0, 0);                     // Back-Right

        beltCenter_ee_frame = new double3(0, -chest_AP_distance / 2.0, 0);

        Debug.Log($"CableTensionPlanner initialized for {matrixCols} cables with {beltSize} belt size (Unity.Mathematics SIMD).");
        return true;
    }

    /// <summary>
    /// Calculates the desired cable tensions based on real-time tracker and force data.
    /// All calculations use Unity.Mathematics SIMD types for performance.
    /// All calculations are performed in the RIGHT-HANDED coordinate system.
    /// </summary>
    /// <param name="endEffectorPose">The 4x4 pose matrix of the end-effector in the world coordinate system.</param>
    /// <param name="desiredWrench">The desired wrench (force and torque) to be applied by the cables.</param>
    /// <param name="robotFramePose">The transformation matrix of the frame-mounted vive tracker</param>
    public double[] CalculateTensions(Matrix4x4 endEffectorPose, Wrench desiredWrench, Matrix4x4 robotFramePose)
    {
        // Convert to Unity.Mathematics double4x4 for SIMD operations (stack allocated)
        double4x4 eeToWorld = ToDouble4x4(endEffectorPose);
        double4x4 robotFrameInv = ToDouble4x4(robotFramePose.inverse);
        double4x4 eeToRobotFrame = mul(robotFrameInv, eeToWorld);

        // Build Structure Matrix using SIMD operations
        for (int i = 0; i < matrixCols; i++)
        {
            // Transform attachment point to robot frame using SIMD
            double3 attachLocal = localAttachmentPoints[i];
            double3 attachRobotFrame = TransformPoint(eeToRobotFrame, attachLocal);

            // Calculate cable direction vector (pulley - attachment)
            double3 cableVec = framePulleyPositions[i] - attachRobotFrame;
            double3 u_i = normalize(cableVec); // SIMD normalized

            // Calculate torque arm: from belt center to attachment point
            double3 r_local = attachLocal - beltCenter_ee_frame;
            double3 r_robotFrame = TransformPoint(eeToRobotFrame, r_local);
            
            // Torque component using SIMD cross product
            double3 torque_i = cross(r_robotFrame, u_i);

            // Fill structure matrix directly
            sMatrix[0, i] = u_i.x;
            sMatrix[1, i] = u_i.y;
            sMatrix[2, i] = u_i.z;
            sMatrix[3, i] = torque_i.x;
            sMatrix[4, i] = torque_i.y;
            sMatrix[5, i] = torque_i.z;
        }

        // Extract wrench components (already double3, no conversion needed)
        double3 desiredForce = desiredWrench.Force;
        double3 desiredTorque = desiredWrench.Torque;
        
        // Update constraint bounds (2-sided with epsilon tolerance)
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

        // Solve QP Problem using Dense IPM (ALGLIB - still allocates internally)
        alglib.minqpsetlc2dense(qpState, sMatrix, lowerWrenchBounds, upperWrenchBounds, 6);
        alglib.minqpsetstartingpoint(qpState, previousSolution); // warm start
        
        alglib.minqpoptimize(qpState);
        alglib.minqpresults(qpState, out tensions, out var report);

        // Update warm-start buffer using Buffer.BlockCopy (faster than Array.Copy for primitives)
        Buffer.BlockCopy(tensions, 0, previousSolution, 0, matrixCols * sizeof(double));

        // Check solver result
        if (report.terminationtype < 0)
        {
            Debug.LogWarning($"QP solver failed: {report.terminationtype}");
            return previousSolution;
        }

        return tensions;
    }

    /// <summary>
    /// Returns pulley positions for visualizer (Vector3 array, pre-allocated during init).
    /// </summary>
    public ReadOnlySpan<Vector3> GetPulleyPositionsInRobotFrame()
    {
        return new ReadOnlySpan<Vector3>(framePulleyPositionsVec3);
    }

    // ============ Unity.Mathematics Helpers (SIMD) ============

    /// <summary>
    /// Convert Unity Matrix4x4 to Unity.Mathematics double4x4.
    /// Stack-allocated, zero heap allocation.
    /// </summary>
    private static double4x4 ToDouble4x4(Matrix4x4 m)
    {
        return new double4x4(
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }

    /// <summary>
    /// Transform point by 4x4 matrix (includes translation).
    /// Uses SIMD mul operation.
    /// </summary>
    private static double3 TransformPoint(double4x4 m, double3 p)
    {
        double4 p4 = new double4(p, 1.0);
        double4 result = mul(m, p4);
        return result.xyz;
    }
}
