using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

public class CableTensionPlannerNative : MonoBehaviour
{
    [Tooltip("Number of cables in the system.")]
    public int numCables = 8;

    [Header("Chest Dimensions")]
    public float chest_AP_distance = 0;
    public float chest_ML_distance = 0;

    [Header("Solver Constraints")]
    public double minTension = 10.0;
    public double maxTension = 200.0;
    public double forceTolerance = 0.01;
    public double torqueTolerance = 1000.0; // High tolerance in original code, adjust as needed

    [Header("ADMM Solver Settings")]
    [Tooltip("Solver Iterations (ADMM converges fast, 25-50 is usually enough)")]
    public int solverIterations = 40;
    [Tooltip("Penalty parameter. Higher = stricter constraint enforcement but slower convergence. 1.0-5.0 is standard.")]
    public double rho = 1.0; 

    // ============ Native Memory ============
    private NativeArray<double3> localAttachmentPoints;
    private NativeArray<double3> framePulleyPositions;
    
    // Solver Workspace
    private NativeArray<double> currentTensions; 
    private NativeArray<double> jobInputWrench;
    private NativeArray<double4x4> jobInputMatrices; 
    private NativeArray<double3> beltCenterRef;
    private NativeArray<double> jobResultError;

    private bool isInitialized = false;
    private JobHandle currentJobHandle;

    public bool Initialize()
    {
        if (chest_AP_distance <= 0 || chest_ML_distance <= 0) return false;

        // Allocate Persistent NativeArrays
        localAttachmentPoints = new NativeArray<double3>(numCables, Allocator.Persistent);
        framePulleyPositions = new NativeArray<double3>(numCables, Allocator.Persistent);
        
        currentTensions = new NativeArray<double>(numCables, Allocator.Persistent);
        jobInputWrench = new NativeArray<double>(6, Allocator.Persistent);
        jobInputMatrices = new NativeArray<double4x4>(1, Allocator.Persistent);
        beltCenterRef = new NativeArray<double3>(1, Allocator.Persistent);
        jobResultError = new NativeArray<double>(1, Allocator.Persistent);

        // --- 1. Setup Geometry (Same as your original) ---
        framePulleyPositions[0] = new double3(-0.8114, 1.6556, 0.9400);   // FR-Top
        framePulleyPositions[1] = new double3(-0.8066, 0.0084, 0.8895);   // FL-Top
        framePulleyPositions[2] = new double3(0.9827, 0.0592, 0.9126);    // BL-Top
        framePulleyPositions[3] = new double3(0.9718, 1.6551, 0.9411);    // BR-Top
        framePulleyPositions[4] = new double3(-0.8084, 1.6496, -0.3060);  // FR-Bot
        framePulleyPositions[5] = new double3(-0.7667, 0.0144, -0.3243);  // FL-Bot
        framePulleyPositions[6] = new double3(0.9748, 0.0681, -0.5438);   // BL-Bot
        framePulleyPositions[7] = new double3(0.9498, 1.6744, -0.5409);   // BR-Bot

        double halfML = chest_ML_distance / 2.0;
        localAttachmentPoints[0] = new double3(-halfML, -chest_AP_distance, 0); 
        localAttachmentPoints[1] = new double3(halfML, -chest_AP_distance, 0); 
        localAttachmentPoints[2] = new double3(halfML, 0, 0); 
        localAttachmentPoints[3] = new double3(-halfML, 0, 0); 
        localAttachmentPoints[4] = new double3(-halfML, -chest_AP_distance, 0); 
        localAttachmentPoints[5] = new double3(halfML, -chest_AP_distance, 0); 
        localAttachmentPoints[6] = new double3(halfML, 0, 0); 
        localAttachmentPoints[7] = new double3(-halfML, 0, 0); 

        beltCenterRef[0] = new double3(0, -chest_AP_distance / 2.0, 0);

        // Warm start with mid-range tensions
        for (int i = 0; i < numCables; i++) currentTensions[i] = 50.0;

        isInitialized = true;
        return true;
    }

    /// <summary>
    /// Solves the QP problem using ADMM.
    /// Returns the Wrench Error (0.0 means solution found within tolerance).
    /// </summary>
    public double CalculateTensions(
        in Matrix4x4 endEffectorPose, 
        in Wrench desiredWrench, 
        in Matrix4x4 robotFramePose,
        NativeSlice<double> resultBuffer)
    {
        // 1. Prepare Inputs
        double4x4 eeToWorld = ToDouble4x4(endEffectorPose);
        double4x4 robotFrameInv = ToDouble4x4(robotFramePose.inverse);
        jobInputMatrices[0] = math.mul(robotFrameInv, eeToWorld);

        jobInputWrench[0] = desiredWrench.Force.x;
        jobInputWrench[1] = desiredWrench.Force.y;
        jobInputWrench[2] = desiredWrench.Force.z;
        jobInputWrench[3] = desiredWrench.Torque.x;
        jobInputWrench[4] = desiredWrench.Torque.y;
        jobInputWrench[5] = desiredWrench.Torque.z;

        // 2. Schedule ADMM Job
        var job = new AdmmSolverJob
        {
            numCables = numCables,
            eeToRobotFrame = jobInputMatrices,
            localAttachments = localAttachmentPoints,
            framePulleys = framePulleyPositions,
            targetWrench = jobInputWrench,
            beltCenterRef = beltCenterRef,
            
            tMin = minTension,
            tMax = maxTension,
            forceTol = forceTolerance,
            torqueTol = torqueTolerance,
            rho = rho,
            iterations = solverIterations,
            
            tensions = currentTensions,
            finalError = jobResultError
        };

        currentJobHandle = job.Schedule();
        currentJobHandle.Complete();

        // 3. Output results
        currentTensions.CopyTo(resultBuffer);
        return jobResultError[0];
    }

    private void OnDestroy()
    {
        if (isInitialized)
        {
            if(!currentJobHandle.IsCompleted) currentJobHandle.Complete();
            if (localAttachmentPoints.IsCreated) localAttachmentPoints.Dispose();
            if (framePulleyPositions.IsCreated) framePulleyPositions.Dispose();
            if (currentTensions.IsCreated) currentTensions.Dispose();
            if (jobInputWrench.IsCreated) jobInputWrench.Dispose();
            if (jobInputMatrices.IsCreated) jobInputMatrices.Dispose();
            if (beltCenterRef.IsCreated) beltCenterRef.Dispose();
            if (jobResultError.IsCreated) jobResultError.Dispose();
        }
    }
    
    private static double4x4 ToDouble4x4(Matrix4x4 m)
    {
        return new double4x4(
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }
}

// ============ THE RIGOROUS SOLVER ============

[BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
struct AdmmSolverJob : IJob
{
    [ReadOnly] public int numCables;
    [ReadOnly] public NativeArray<double4x4> eeToRobotFrame;
    [ReadOnly] public NativeArray<double3> localAttachments;
    [ReadOnly] public NativeArray<double3> framePulleys;
    [ReadOnly] public NativeArray<double> targetWrench;
    [ReadOnly] public NativeArray<double3> beltCenterRef;

    [ReadOnly] public double tMin;
    [ReadOnly] public double tMax;
    [ReadOnly] public double forceTol;
    [ReadOnly] public double torqueTol;
    [ReadOnly] public double rho; // Penalty parameter
    [ReadOnly] public int iterations;

    public NativeArray<double> tensions; // In/Out (Warm Start)
    public NativeArray<double> finalError;

    public unsafe void Execute()
    {
        // --- 1. SETUP STRUCTURE MATRIX (S) ---
        // S is 6 x numCables.
        // Stack allocation for speed. Max 8 cables supported here (6*8 = 48 doubles).
        // If you increase cables, increase this size.
        double* S = stackalloc double[6 * 8]; 

        double4x4 T = eeToRobotFrame[0];
        double3 beltCenter = beltCenterRef[0];
        
        for (int i = 0; i < numCables; i++)
        {
            double4 p4 = new double4(localAttachments[i], 1.0);
            double3 attachPos = math.mul(T, p4).xyz;
            double3 diff = framePulleys[i] - attachPos;
            double len = math.length(diff);
            double3 u = (len > 1e-6) ? diff / len : double3.zero;

            double4 r_local = new double4(localAttachments[i] - beltCenter, 0.0);
            double3 r = math.mul(T, r_local).xyz; 
            double3 tau = math.cross(r, u); 

            // Column Major or Row Major? Let's index manually: S[row * 8 + col]
            S[0 * 8 + i] = u.x;
            S[1 * 8 + i] = u.y;
            S[2 * 8 + i] = u.z;
            S[3 * 8 + i] = tau.x;
            S[4 * 8 + i] = tau.y;
            S[5 * 8 + i] = tau.z;
        }

        // --- 2. PRECOMPUTE ADMM MATRICES ---
        // Problem: Minimize (1/2)x^Tx + I_box(x) + I_wrench(Sx)
        // x-update requires solving linear system: (I + rho * S^T * S) * x = ...
        // We compute A = (I + rho * S^T * S) once, then invert it using Gaussian Elimination.
        
        // A is 8x8 matrix
        double* A = stackalloc double[8 * 8];
        double* A_inv = stackalloc double[8 * 8];

        // Fill A = Identity + rho * S^T * S
        for (int r = 0; r < numCables; r++)
        {
            for (int c = 0; c < numCables; c++)
            {
                double sumSTS = 0;
                for (int k = 0; k < 6; k++) sumSTS += S[k * 8 + r] * S[k * 8 + c];
                
                double val = (rho * sumSTS);
                if (r == c) val += 1.0; // Add Identity
                
                A[r * 8 + c] = val;
            }
        }

        // Invert A (8x8 is small enough for naive Gaussian Elimination on stack)
        InvertMatrix8x8(A, A_inv, numCables);

        // --- 3. ADMM VARIABLES ---
        // x = tensions (already in array)
        // z = auxiliary variable (target wrench)
        // u = dual variable
        double* z = stackalloc double[6];
        double* u = stackalloc double[6];
        double* rhs = stackalloc double[8]; // Right hand side for linear solve

        // Initialize z, u to zero
        for(int k=0; k<6; k++) { z[k] = 0; u[k] = 0; }

        // --- 4. ADMM LOOP ---
        for (int iter = 0; iter < iterations; iter++)
        {
            // --- Step A: x-update ---
            // Solve (I + rho S^T S) x = rho S^T (z - u)
            // rhs = rho * S^T * (z - u)
            
            for (int i = 0; i < numCables; i++)
            {
                double sum = 0;
                for (int k = 0; k < 6; k++)
                {
                    sum += S[k * 8 + i] * (z[k] - u[k]);
                }
                rhs[i] = rho * sum;
            }

            // Multiply x = A_inv * rhs
            // Note: We write directly to tensions array
            for (int r = 0; r < numCables; r++)
            {
                double val = 0;
                for (int c = 0; c < numCables; c++)
                {
                    val += A_inv[r * 8 + c] * rhs[c];
                }
                // --- Step B: Box Projection of x (Clamp Tensions) ---
                tensions[r] = math.clamp(val, tMin, tMax);
            }

            // --- Step C: z-update ---
            // z = Project_Wrench(Sx + u)
            // 1. Calculate v = S*x
            for (int k = 0; k < 6; k++)
            {
                double sx_k = 0;
                for (int i = 0; i < numCables; i++) sx_k += S[k * 8 + i] * tensions[i];
                
                // 2. Target = sx_k + u_k
                double val = sx_k + u[k];

                // 3. Project onto Wrench Box (Target +/- Tolerance)
                double wTarget = targetWrench[k];
                double tol = (k < 3) ? forceTol : torqueTol;
                z[k] = math.clamp(val, wTarget - tol, wTarget + tol);
                
                // --- Step D: u-update ---
                // u = u + Sx - z
                u[k] = u[k] + sx_k - z[k];
            }
        }

        // --- 5. CALCULATE FINAL ERROR ---
        // Error = Distance from S*x to the Valid Wrench Box
        double totalErrorSq = 0;
        for (int k = 0; k < 6; k++)
        {
            double sx_k = 0;
            for (int i = 0; i < numCables; i++) sx_k += S[k * 8 + i] * tensions[i];
            
            double wTarget = targetWrench[k];
            double tol = (k < 3) ? forceTol : torqueTol;
            
            // Distance to interval [min, max]
            double dist = 0;
            if (sx_k < wTarget - tol) dist = (wTarget - tol) - sx_k;
            else if (sx_k > wTarget + tol) dist = sx_k - (wTarget + tol);
            
            totalErrorSq += dist * dist;
        }
        finalError[0] = totalErrorSq;
    }

    // Helper: Simple Gaussian Elimination for 8x8 Matrix Inversion
    // Modifies input matrices directly.
    private unsafe void InvertMatrix8x8(double* A, double* Inv, int N)
    {
        // Initialize Inv to Identity
        for (int i = 0; i < N * N; i++) Inv[i] = 0.0;
        for (int i = 0; i < N; i++) Inv[i * 8 + i] = 1.0;

        // Gauss-Jordan
        for (int i = 0; i < N; i++)
        {
            double pivot = A[i * 8 + i];
            // If pivot is too small, matrix is singular (bad config), clamp to epsilon
            if (math.abs(pivot) < 1e-9) pivot = 1e-9; 
            
            double invPivot = 1.0 / pivot;

            // Normalize row i
            for (int j = 0; j < N; j++)
            {
                A[i * 8 + j] *= invPivot;
                Inv[i * 8 + j] *= invPivot;
            }

            // Eliminate other rows
            for (int k = 0; k < N; k++)
            {
                if (k != i)
                {
                    double factor = A[k * 8 + i];
                    for (int j = 0; j < N; j++)
                    {
                        A[k * 8 + j] -= factor * A[i * 8 + j];
                        Inv[k * 8 + j] -= factor * Inv[i * 8 + j];
                    }
                }
            }
        }
    }
}