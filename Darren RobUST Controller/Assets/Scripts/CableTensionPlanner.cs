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
/// - Pre-allocates all arrays during construction - no runtime allocations
/// - ALGLIB still allocates internally (~9.2 KB per solve) - see CableTensionPlannerNative for zero-alloc version
/// - Reuses QP state with warm-start for faster convergence
/// </summary>
public class CableTensionPlanner
{
    // Reference to shared robot description (injected during Initialize)
    private RobUSTDescription robot;

    // Pre-allocated QP matrices (allocated once, reused every calculation)
    private double[] tensions;
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

    /// <summary>
    /// Constructs and initializes the tension planner using the robot description.
    /// All allocations happen here - zero runtime allocations.
    /// </summary>
    /// <param name="robotDescription">RobUST description (owned by RobotController)</param>
    public CableTensionPlanner(RobUSTDescription robotDescription)
    {
        robot = robotDescription ?? throw new ArgumentNullException(nameof(robotDescription));
        int numCables = robot.NumCables;

        // Pre-allocate QP-specific data structures
        tensions = new double[numCables];
        quadratic = new double[numCables, numCables];
        linear = new double[numCables];
        sMatrix = new double[6, numCables]; // 6xnumCables structure matrix
        tensionLower = new double[numCables]; // box constraints arrays
        tensionUpper = new double[numCables];
        lowerWrenchBounds = new double[6]; // linear constraints arrays
        upperWrenchBounds = new double[6];

        // Initialize quadratic matrix: P = 2*Identity (constant, never changes)
        for (int i = 0; i < numCables; i++)
            quadratic[i, i] = 2.0;
            
        // Initialize tension bounds (constant, never changes)
        for (int i = 0; i < numCables; i++)
        {
            tensionLower[i] = minTension;  // Minimum tension
            tensionUpper[i] = maxTension; // Maximum tension
        }

        // Initialize previousSolution with a reasonable default
        previousSolution = new double[numCables];
        for (int i = 0; i < numCables; i++)
            previousSolution[i] = 10.0; // default tension value
        
        // initialize qp solver
        alglib.minqpcreate(numCables, out qpState); // Initialize QP state once and reuse it for subsequent solves
        alglib.minqpsetalgodenseipm(qpState, 1e-6);  // Stopping tolerance: 1e-6
        alglib.minqpsetquadraticterm(qpState, quadratic, true);
        alglib.minqpsetlinearterm(qpState, linear);
        alglib.minqpsetbc(qpState, tensionLower, tensionUpper);
    }

    /// <summary>
    /// Calculates the desired cable tensions based on real-time tracker and force data.
    /// All calculations use Unity.Mathematics SIMD types for performance.
    /// </summary>
    /// <param name="endEffectorPose">The 4x4 pose matrix of the end-effector in the world coordinate system.</param>
    /// <param name="desiredWrench">The desired wrench (force and torque) to be applied by the cables.</param>
    /// <param name="robotFramePose">The transformation matrix of the frame-mounted vive tracker</param>
    public double[] CalculateTensions(Matrix4x4 endEffectorPose, Wrench desiredWrench, Matrix4x4 robotFramePose)
    {
        // Convert to Unity.Mathematics double4x4 for SIMD operations (stack allocated)
        double4x4 eeToWorld = ToDouble4x4(endEffectorPose);
        double4x4 robotFrameInv = math.fastinverse(ToDouble4x4(robotFramePose));
        double4x4 eeToRobotFrame = mul(robotFrameInv, eeToWorld);

        int numCables = robot.NumCables;
        
        // Build Structure Matrix using SIMD operations
        for (int i = 0; i < numCables; i++)
        {
            // Transform attachment point to robot frame using SIMD
            double3 attachLocal = robot.LocalAttachmentPoints[i];
            double3 attachRobotFrame = TransformPoint(eeToRobotFrame, attachLocal);

            // Calculate cable direction vector (pulley - attachment)
            double3 cableVec = robot.FramePulleyPositions[i] - attachRobotFrame;
            double3 u_i = normalize(cableVec); // SIMD normalized

            // Calculate torque arm: from belt center to attachment point
            double3 r_local = attachLocal - robot.BeltCenter_EE_Frame;
            double3 r_robotFrame = TransformVector(eeToRobotFrame, r_local);
            
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

        // Check solver result
        if (report.terminationtype < 0)
        {
            //Debug.LogWarning($"QP solver failed: {report.terminationtype}");
            return previousSolution;
        }
        
        // Update warm-start buffer using Buffer.BlockCopy (faster than Array.Copy for primitives)
        Buffer.BlockCopy(tensions, 0, previousSolution, 0, robot.NumCables * sizeof(double));
        return tensions;
    }

    /// <summary>
    /// Returns pulley positions for visualizer (Vector3 array, pre-allocated in description).
    /// </summary>
    public ReadOnlySpan<Vector3> GetPulleyPositionsInRobotFrame()
    {
        return new ReadOnlySpan<Vector3>(robot.FramePulleyPositionsVec3);
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

    /// <summary>
    /// Transform vector by 4x4 matrix (rotation only, no translation).
    /// Uses SIMD mul operation.
    /// </summary>
    private static double3 TransformVector(double4x4 m, double3 v)
    {
        double4 v4 = new double4(v, 0.0);   // w = 0
        double4 result = mul(m, v4);
        return result.xyz;
    }

}
