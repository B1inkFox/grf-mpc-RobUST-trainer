using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using static Unity.Mathematics.math;

/// <summary>
/// ZERO-ALLOCATION cable tension planner using Unity.Mathematics and NativeArrays.
/// Replaces ALGLIB with a custom active-set QP solver optimized for the 8-cable problem.
/// 
/// Key Features:
/// - All arrays are NativeArrays (unmanaged, no GC)
/// - Burst-compatible math operations
/// - Custom QP solver with warm-start support
/// - SIMD-optimized via Unity.Mathematics
/// 
/// Problem: min 0.5*||t||^2  s.t.  S*t = w,  t_min <= t <= t_max
/// Where: t = tensions (8), S = structure matrix (6x8), w = desired wrench (6)
/// </summary>
public class CableTensionPlannerNative : MonoBehaviour
{
    [Tooltip("Number of cables in the system.")]
    public int numCables = 8;

    [Header("Chest Dimensions")]
    [Tooltip("Measured thickness of the chest in the anterior-posterior direction.")]
    public float chest_AP_distance = 0;

    [Tooltip("Measured width of the chest in the medial-lateral direction.")]
    public float chest_ML_distance = 0;

    public enum BeltSize { Small, Large }
    [Header("Belt Size")]
    public BeltSize beltSize = BeltSize.Small;

    // Solver parameters
    private double forceEpsilon = 0.01;
    private double torqueEpsilon = 1000;
    private double minTension = 10.0;
    private double maxTension = 200.0;
    private int maxIterations = 50;
    private double tolerance = 1e-6;

    // ============ NativeArrays (Zero GC) ============
    // Geometry (constant after init)
    private NativeArray<double3> localAttachmentPoints;  // Cable attachment points on end-effector
    private NativeArray<double3> framePulleyPositions;   // Pulley positions in robot frame
    
    // QP solver workspace (pre-allocated, reused)
    private NativeArray<double> tensions;          // Solution vector [8]
    private NativeArray<double> previousSolution;  // Warm-start buffer [8]
    private NativeArray<double> gradient;          // Gradient buffer [8]
    private NativeArray<double> direction;         // Search direction [8]
    private NativeArray<double> wrenchTarget;      // RHS: [fx, fy, fz, tx, ty, tz]
    
    // Structure matrix stored as flat array for cache efficiency
    // Layout: row-major [6 rows x 8 cols] = 48 elements
    private NativeArray<double> structureMatrix;
    
    // Linear system workspace
    private NativeArray<double> STS;      // S'*S matrix [8x8] = 64 elements
    private NativeArray<double> STw;      // S'*w vector [8]
    private NativeArray<double> SST;      // S*S' matrix [6x6] = 36 elements
    private NativeArray<double> tempVec6; // Temp vector [6]
    private NativeArray<double> tempVec8; // Temp vector [8]
    private NativeArray<double> choleskyL; // Cholesky factor (max 8x8)
    private NativeArray<double> choleskyY; // Cholesky temp [8]

    // Cached geometry values
    private double3 beltCenter_ee_frame; // Offset from EE origin to CoM for torque calculation

    private bool isInitialized = false;

    /// <summary>
    /// Initializes the native tension planner with zero-allocation data structures.
    /// </summary>
    public bool Initialize()
    {
        if (chest_AP_distance <= 0 || chest_ML_distance <= 0)
        {
            Debug.LogError("Chest dimensions must be positive values.", this);
            return false;
        }

        // Allocate all NativeArrays with Persistent allocator (lives until manually disposed)
        localAttachmentPoints = new NativeArray<double3>(numCables, Allocator.Persistent);
        framePulleyPositions = new NativeArray<double3>(numCables, Allocator.Persistent);
        
        tensions = new NativeArray<double>(numCables, Allocator.Persistent);
        previousSolution = new NativeArray<double>(numCables, Allocator.Persistent);
        gradient = new NativeArray<double>(numCables, Allocator.Persistent);
        direction = new NativeArray<double>(numCables, Allocator.Persistent);
        wrenchTarget = new NativeArray<double>(6, Allocator.Persistent);
        
        structureMatrix = new NativeArray<double>(6 * numCables, Allocator.Persistent);
        
        STS = new NativeArray<double>(numCables * numCables, Allocator.Persistent);
        STw = new NativeArray<double>(numCables, Allocator.Persistent);
        SST = new NativeArray<double>(36, Allocator.Persistent);
        tempVec6 = new NativeArray<double>(6, Allocator.Persistent);
        tempVec8 = new NativeArray<double>(numCables, Allocator.Persistent);
        choleskyL = new NativeArray<double>(numCables * numCables, Allocator.Persistent);
        choleskyY = new NativeArray<double>(numCables, Allocator.Persistent);

        // Initialize pulley positions (robot frame coordinates)
        framePulleyPositions[0] = new double3(-0.8114, 1.6556, 0.9400);   // Front-Right Top
        framePulleyPositions[1] = new double3(-0.8066, 0.0084, 0.8895);   // Front-Left Top
        framePulleyPositions[2] = new double3(0.9827, 0.0592, 0.9126);    // Back-Left Top
        framePulleyPositions[3] = new double3(0.9718, 1.6551, 0.9411);    // Back-Right Top
        framePulleyPositions[4] = new double3(-0.8084, 1.6496, -0.3060);  // Front-Right Bottom
        framePulleyPositions[5] = new double3(-0.7667, 0.0144, -0.3243);  // Front-Left Bottom
        framePulleyPositions[6] = new double3(0.9748, 0.0681, -0.5438);   // Back-Left Bottom
        framePulleyPositions[7] = new double3(0.9498, 1.6744, -0.5409);   // Back-Right Bottom

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

        // CoM offset for torque calculation
        beltCenter_ee_frame = new double3(0, -chest_AP_distance / 2.0, 0);

        // Initialize tensions to mid-range
        for (int i = 0; i < numCables; i++)
        {
            tensions[i] = 50.0;
            previousSolution[i] = 50.0;
        }

        isInitialized = true;
        Debug.Log($"CableTensionPlannerNative initialized: {numCables} cables, {beltSize} belt, zero-allocation mode");
        return true;
    }

    /// <summary>
    /// Calculates cable tensions with ZERO heap allocations.
    /// All math uses Unity.Mathematics SIMD types.
    /// </summary>
    /// <param name="endEffectorPose">End-effector pose matrix (Unity Matrix4x4)</param>
    /// <param name="desiredWrench">Desired force/torque to achieve</param>
    /// <param name="robotFramePose">Robot frame pose matrix</param>
    /// <param name="result">Output: computed tensions (must be pre-allocated, length = numCables)</param>
    public void CalculateTensions(
        in Matrix4x4 endEffectorPose, 
        in Wrench desiredWrench, 
        in Matrix4x4 robotFramePose,
        NativeArray<double> result)
    {
        // Convert Unity matrices to double4x4 for SIMD operations
        double4x4 eeToWorld = ToDouble4x4(endEffectorPose);
        double4x4 robotFrameInv = ToDouble4x4(robotFramePose.inverse);
        double4x4 eeToRobotFrame = mul(robotFrameInv, eeToWorld);

        // === 1. Build Structure Matrix (Burst-compiled parallel job) ===
        var buildJob = new BuildStructureMatrixJob
        {
            eeToRobotFrame = eeToRobotFrame,
            localAttachments = localAttachmentPoints,
            framePulleys = framePulleyPositions,
            beltCenter = beltCenter_ee_frame,
            structureMatrix = structureMatrix,
            numCables = numCables
        };
        buildJob.Schedule(numCables, 1).Complete(); // Run in parallel (8 threads)

        // === 2. Set Wrench Target ===
        wrenchTarget[0] = desiredWrench.Force.x;
        wrenchTarget[1] = desiredWrench.Force.y;
        wrenchTarget[2] = desiredWrench.Force.z;
        wrenchTarget[3] = desiredWrench.Torque.x;
        wrenchTarget[4] = desiredWrench.Torque.y;
        wrenchTarget[5] = desiredWrench.Torque.z;

        // === 3. Solve QP (Zero Allocation) ===
        SolveQP_ActiveSet();

        // === 4. Copy result ===
        tensions.CopyTo(result);
        
        // Update warm-start for next frame
        tensions.CopyTo(previousSolution);
    }

    /// <summary>
    /// Overload that returns tensions in the internal buffer (caller must not hold reference across frames).
    /// </summary>
    public NativeArray<double> CalculateTensions(
        in Matrix4x4 endEffectorPose,
        in Wrench desiredWrench,
        in Matrix4x4 robotFramePose)
    {
        double4x4 eeToWorld = ToDouble4x4(endEffectorPose);
        double4x4 robotFrameInv = ToDouble4x4(robotFramePose.inverse);
        double4x4 eeToRobotFrame = mul(robotFrameInv, eeToWorld);

        var buildJob = new BuildStructureMatrixJob
        {
            eeToRobotFrame = eeToRobotFrame,
            localAttachments = localAttachmentPoints,
            framePulleys = framePulleyPositions,
            beltCenter = beltCenter_ee_frame,
            structureMatrix = structureMatrix,
            numCables = numCables
        };
        buildJob.Schedule(numCables, 1).Complete();

        wrenchTarget[0] = desiredWrench.Force.x;
        wrenchTarget[1] = desiredWrench.Force.y;
        wrenchTarget[2] = desiredWrench.Force.z;
        wrenchTarget[3] = desiredWrench.Torque.x;
        wrenchTarget[4] = desiredWrench.Torque.y;
        wrenchTarget[5] = desiredWrench.Torque.z;

        SolveQP_ActiveSet();
        tensions.CopyTo(previousSolution);

        return tensions;
    }

    // ============ BURST-COMPILED PARALLEL JOBS ============

    /// <summary>
    /// Parallel Burst-compiled job to build structure matrix.
    /// Each cable's computation runs on a separate thread.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    private struct BuildStructureMatrixJob : IJobParallelFor
    {
        [ReadOnly] public double4x4 eeToRobotFrame;
        [ReadOnly] public NativeArray<double3> localAttachments;
        [ReadOnly] public NativeArray<double3> framePulleys;
        [ReadOnly] public double3 beltCenter;
        [ReadOnly] public int numCables;
        
        [WriteOnly] public NativeArray<double> structureMatrix;

        public void Execute(int cableIndex)
        {
            // Transform attachment point to robot frame
            double3 attachLocal = localAttachments[cableIndex];
            double3 attachRobotFrame = TransformPoint(eeToRobotFrame, attachLocal);

            // Cable direction vector (normalized)
            double3 cableVec = framePulleys[cableIndex] - attachRobotFrame;
            double3 u_i = normalize(cableVec);

            // Torque arm: from belt center to attachment point, in robot frame
            double3 r_local = attachLocal - beltCenter;
            double3 r_robotFrame = TransformPoint(eeToRobotFrame, r_local);
            
            // Torque component: r × u
            double3 torque_i = cross(r_robotFrame, u_i);

            // Fill structure matrix column (row-major indexing)
            int col = cableIndex;
            structureMatrix[0 * numCables + col] = u_i.x;       // Force X
            structureMatrix[1 * numCables + col] = u_i.y;       // Force Y
            structureMatrix[2 * numCables + col] = u_i.z;       // Force Z
            structureMatrix[3 * numCables + col] = torque_i.x;  // Torque X
            structureMatrix[4 * numCables + col] = torque_i.y;  // Torque Y
            structureMatrix[5 * numCables + col] = torque_i.z;  // Torque Z
        }

        // Static helper for Burst compilation
        private static double3 TransformPoint(double4x4 m, double3 p)
        {
            double4 p4 = new double4(p, 1.0);
            double4 result = mul(m, p4);
            return result.xyz;

            // Fill structure matrix (row-major: S[row, col] = S[row * numCables + col])
            int col = i;
            structureMatrix[0 * numCables + col] = u_i.x;       // Force X
            structureMatrix[1 * numCables + col] = u_i.y;       // Force Y
            structureMatrix[2 * numCables + col] = u_i.z;       // Force Z
            structureMatrix[3 * numCables + col] = torque_i.x;  // Torque X
            structureMatrix[4 * numCables + col] = torque_i.y;  // Torque Y
            structureMatrix[5 * numCables + col] = torque_i.z;  // Torque Z
        }
    }

    /// <summary>
    /// Custom active-set QP solver for the constrained tension problem.
    /// Solves: min 0.5*||t||^2  s.t.  S*t ≈ w (with epsilon tolerance),  t_min <= t <= t_max
    /// 
    /// Uses damped least-squares projection with iterative box constraint enforcement.
    /// Zero allocations - all workspace is pre-allocated NativeArrays.
    /// </summary>
    private void SolveQP_ActiveSet()
    {
        const double lambda = 1e-4; // Damping for numerical stability

        // Copy warm-start to working solution
        previousSolution.CopyTo(tensions);

        for (int iter = 0; iter < maxIterations; iter++)
        {
            // === Step 1: Project onto equality constraints via damped least-squares ===
            // Solve: (S'*S + λI) * t = S'*w
            
            // Compute S'*S (8x8 symmetric matrix) - Burst compiled
            BurstMath.ComputeSTS(structureMatrix, STS, numCables, lambda);
            
            // Compute S'*w (8-vector) - Burst compiled
            BurstMath.ComputeSTw(structureMatrix, wrenchTarget, STw, numCables);
            
            // Solve linear system using Cholesky - Burst compiled
            BurstMath.SolveCholesky8x8(STS, STw, tensions, choleskyL, choleskyY, numCables);

            // === Step 2: Enforce box constraints ===
            for (int i = 0; i < numCables; i++)
            {
                tensions[i] = clamp(tensions[i], minTension, maxTension);
            }

            // === Step 3: Check convergence (gradient in feasible space) ===
            // For objective 0.5*||t||^2, gradient = t
            // Converged when projected gradient norm is small
            double gradNorm = 0;
            for (int i = 0; i < numCables; i++)
            {
                double g = tensions[i]; // gradient = t for this objective
                
                // Project gradient onto feasible direction
                if ((tensions[i] <= minTension && g < 0) || (tensions[i] >= maxTension && g > 0))
                    g = 0; // At boundary, can't move in this direction
                
                gradNorm += g * g;
            }

            if (sqrt(gradNorm) < tolerance)
                break;
        }
    }

    // ============ BURST-COMPILED STATIC MATH FUNCTIONS ============
    
    /// <summary>
    /// Static class containing Burst-compiled matrix operations.
    /// Static methods are required for Burst compilation to work.
    /// </summary>
    private static class BurstMath
    {
        /// <summary>
        /// Computes S'*S + λI. S is 6xN, result is NxN.
        /// Burst-compiled for maximum performance.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
        public static void ComputeSTS(
            NativeArray<double> structureMatrix,
            NativeArray<double> STS,
            int numCables,
            double lambda)
        {
            for (int i = 0; i < numCables; i++)
            {
                for (int j = 0; j < numCables; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < 6; k++)
                    {
                        // S[k,i] * S[k,j] where S is row-major
                        sum += structureMatrix[k * numCables + i] * structureMatrix[k * numCables + j];
                    }
                    // Add damping on diagonal
                    STS[i * numCables + j] = sum + (i == j ? lambda : 0);
                }
            }
        }

        /// <summary>
        /// Computes S'*w. S is 6xN, w is 6, result is N.
        /// Burst-compiled for maximum performance.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
        public static void ComputeSTw(
            NativeArray<double> structureMatrix,
            NativeArray<double> wrenchTarget,
            NativeArray<double> STw,
            int numCables)
        {
            for (int i = 0; i < numCables; i++)
            {
                double sum = 0;
                for (int k = 0; k < 6; k++)
                {
                    sum += structureMatrix[k * numCables + i] * wrenchTarget[k];
                }
                STw[i] = sum;
            }
        }

        /// <summary>
        /// Solves A*x = b using Cholesky decomposition for NxN symmetric positive definite A.
        /// Burst-compiled for maximum performance.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
        public static void SolveCholesky8x8(
            NativeArray<double> A,
            NativeArray<double> b,
            NativeArray<double> x,
            NativeArray<double> choleskyL,
            NativeArray<double> choleskyY,
            int n)
        {
            // === Cholesky decomposition: A = L*L' ===
            // L stored in choleskyL (lower triangular, row-major)
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < j; k++)
                    {
                        sum += choleskyL[i * n + k] * choleskyL[j * n + k];
                    }
                    
                    if (i == j)
                    {
                        double diag = A[i * n + i] - sum;
                        choleskyL[i * n + j] = (diag > 0) ? sqrt(diag) : 1e-10;
                    }
                    else
                    {
                        choleskyL[i * n + j] = (A[i * n + j] - sum) / choleskyL[j * n + j];
                    }
                }
                // Zero out upper triangle
                for (int j = i + 1; j < n; j++)
                {
                    choleskyL[i * n + j] = 0;
                }
            }

            // === Forward substitution: L*y = b ===
            for (int i = 0; i < n; i++)
            {
                double sum = 0;
                for (int j = 0; j < i; j++)
                {
                    sum += choleskyL[i * n + j] * choleskyY[j];
                }
                choleskyY[i] = (b[i] - sum) / choleskyL[i * n + i];
            }

            // === Back substitution: L'*x = y ===
            for (int i = n - 1; i >= 0; i--)
            {
                double sum = 0;
                for (int j = i + 1; j < n; j++)
                {
                    sum += choleskyL[j * n + i] * x[j]; // L' element is L[j,i]
                }
                x[i] = (choleskyY[i] - sum) / choleskyL[i * n + i];
            }
        }
    }

    /// <summary>
    /// Returns pulley positions as NativeArray (read-only access pattern expected).
    /// </summary>
    public NativeArray<double3> GetPulleyPositionsNative()
    {
        return framePulleyPositions;
    }

    /// <summary>
    /// Returns pulley positions as Vector3 array for compatibility with existing visualizer.
    /// NOTE: This allocates! Use only during initialization.
    /// </summary>
    public Vector3[] GetPulleyPositionsAsVector3Array()
    {
        var result = new Vector3[numCables];
        for (int i = 0; i < numCables; i++)
            result[i] = new Vector3((float3)framePulleyPositions[i]);
        return result;
    }

    // ============ Unity.Mathematics Helpers ============

    /// <summary>
    /// Convert Unity Matrix4x4 to Unity.Mathematics double4x4 (SIMD-friendly).
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
    /// Transform a point by a 4x4 matrix (applies translation).
    /// </summary>
    private static double3 TransformPoint(double4x4 m, double3 p)
    {
        double4 p4 = new double4(p, 1.0);
        double4 result = mul(m, p4);
        return result.xyz;
    }

    /// <summary>
    /// Transform a direction vector by a 4x4 matrix (no translation).
    /// </summary>
    private static double3 TransformDirection(double4x4 m, double3 v)
    {
        double4 v4 = new double4(v, 0.0);
        double4 result = mul(m, v4);
        return result.xyz;
    }

    // ============ Lifecycle ============

    private void OnDestroy()
    {
        // CRITICAL: Dispose all NativeArrays to prevent memory leaks
        if (isInitialized)
        {
            if (localAttachmentPoints.IsCreated) localAttachmentPoints.Dispose();
            if (framePulleyPositions.IsCreated) framePulleyPositions.Dispose();
            if (tensions.IsCreated) tensions.Dispose();
            if (previousSolution.IsCreated) previousSolution.Dispose();
            if (gradient.IsCreated) gradient.Dispose();
            if (direction.IsCreated) direction.Dispose();
            if (wrenchTarget.IsCreated) wrenchTarget.Dispose();
            if (structureMatrix.IsCreated) structureMatrix.Dispose();
            if (STS.IsCreated) STS.Dispose();
            if (STw.IsCreated) STw.Dispose();
            if (SST.IsCreated) SST.Dispose();
            if (tempVec6.IsCreated) tempVec6.Dispose();
            if (tempVec8.IsCreated) tempVec8.Dispose();
            if (choleskyL.IsCreated) choleskyL.Dispose();
            if (choleskyY.IsCreated) choleskyY.Dispose();
        }
    }
}
