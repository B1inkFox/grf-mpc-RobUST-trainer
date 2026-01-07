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
    [Tooltip("Measured thickness of the chest in the anterior-posterior direction.")]
    public float chest_AP_distance = 0;

    [Tooltip("Measured width of the chest in the medial-lateral direction.")]
    public float chest_ML_distance = 0;

    public enum BeltSize { Small, Large }
    [Header("Belt Size")]
    public BeltSize beltSize = BeltSize.Small;

    [Header("Solver Parameters")]
    public double minTension = 10.0;
    public double maxTension = 200.0;
    public int maxIterations = 64; 
    public double learningRate = 0.4;   // Alpha for Gradient Descent
    public double regularization = 1e-4; // Lambda: minimizes excess tension

    // ============ Native Memory ============
    // Geometry (constant after init)
    private NativeArray<double3> localAttachmentPoints;
    private NativeArray<double3> framePulleyPositions;
    
    // Solver Workspace
    private NativeArray<double> currentTensions; 
    private NativeArray<double> jobInputWrench;
    private NativeArray<double4x4> jobInputMatrices; // [0] = eeToRobotFrame
    private NativeArray<double3> beltCenterRef;      // [0] = beltCenter

    private bool isInitialized = false;
    private JobHandle currentJobHandle;

    public bool Initialize()
    {
        if (chest_AP_distance <= 0 || chest_ML_distance <= 0)
        {
            Debug.LogError("Chest dimensions must be positive values.", this);
            return false;
        }

        // Allocate Persistent NativeArrays
        localAttachmentPoints = new NativeArray<double3>(numCables, Allocator.Persistent);
        framePulleyPositions = new NativeArray<double3>(numCables, Allocator.Persistent);
        
        currentTensions = new NativeArray<double>(numCables, Allocator.Persistent);
        jobInputWrench = new NativeArray<double>(6, Allocator.Persistent);
        jobInputMatrices = new NativeArray<double4x4>(1, Allocator.Persistent);
        beltCenterRef = new NativeArray<double3>(1, Allocator.Persistent);

        // --- 1. Restore Your Original Pulley Coordinates ---
        framePulleyPositions[0] = new double3(-0.8114, 1.6556, 0.9400);   // Front-Right Top
        framePulleyPositions[1] = new double3(-0.8066, 0.0084, 0.8895);   // Front-Left Top
        framePulleyPositions[2] = new double3(0.9827, 0.0592, 0.9126);    // Back-Left Top
        framePulleyPositions[3] = new double3(0.9718, 1.6551, 0.9411);    // Back-Right Top
        framePulleyPositions[4] = new double3(-0.8084, 1.6496, -0.3060);  // Front-Right Bottom
        framePulleyPositions[5] = new double3(-0.7667, 0.0144, -0.3243);  // Front-Left Bottom
        framePulleyPositions[6] = new double3(0.9748, 0.0681, -0.5438);   // Back-Left Bottom
        framePulleyPositions[7] = new double3(0.9498, 1.6744, -0.5409);   // Back-Right Bottom

        // --- 2. Restore Your Original Attachment Logic ---
        double halfML = chest_ML_distance / 2.0;

        localAttachmentPoints[0] = new double3(-halfML, -chest_AP_distance, 0);  // Front-Right
        localAttachmentPoints[1] = new double3(halfML, -chest_AP_distance, 0);   // Front-Left
        localAttachmentPoints[2] = new double3(halfML, 0, 0);                    // Back-Left 
        localAttachmentPoints[3] = new double3(-halfML, 0, 0);                   // Back-Right 
        // Bottom cables
        localAttachmentPoints[4] = new double3(-halfML, -chest_AP_distance, 0);  // Front-Right
        localAttachmentPoints[5] = new double3(halfML, -chest_AP_distance, 0);   // Front-Left
        localAttachmentPoints[6] = new double3(halfML, 0, 0);                    // Back-Left
        localAttachmentPoints[7] = new double3(-halfML, 0, 0);                   // Back-Right

        // CoM offset
        beltCenterRef[0] = new double3(0, -chest_AP_distance / 2.0, 0);

        // Warm start tensions
        for (int i = 0; i < numCables; i++) currentTensions[i] = 50.0;

        isInitialized = true;
        Debug.Log($"CableTensionPlannerNative Optimized: {numCables} cables initialized.");
        return true;
    }

    /// <summary>
    /// Drop-in replacement for synchronous calculation.
    /// Schedules the optimized job and completes it immediately.
    /// </summary>
    public void CalculateTensions(
        in Matrix4x4 endEffectorPose, 
        in Wrench desiredWrench, 
        in Matrix4x4 robotFramePose,
        NativeArray<double> result)
    {
        ScheduleJob(endEffectorPose, desiredWrench, robotFramePose);
        currentJobHandle.Complete();
        currentTensions.CopyTo(result);
    }

    /// <summary>
    /// Overload that returns internal buffer (be careful with lifetime).
    /// </summary>
    public NativeArray<double> CalculateTensions(
        in Matrix4x4 endEffectorPose,
        in Wrench desiredWrench,
        in Matrix4x4 robotFramePose)
    {
        ScheduleJob(endEffectorPose, desiredWrench, robotFramePose);
        currentJobHandle.Complete();
        return currentTensions;
    }

    /// <summary>
    /// Advanced: Use this if you want to run physics in parallel with other Update logic.
    /// </summary>
    public void ScheduleAsync(in Matrix4x4 endEffectorPose, in Wrench desiredWrench, in Matrix4x4 robotFramePose)
    {
        ScheduleJob(endEffectorPose, desiredWrench, robotFramePose);
    }

    private void ScheduleJob(in Matrix4x4 endEffectorPose, in Wrench desiredWrench, in Matrix4x4 robotFramePose)
    {
        // Convert Matrices
        double4x4 eeToWorld = ToDouble4x4(endEffectorPose);
        double4x4 robotFrameInv = ToDouble4x4(robotFramePose.inverse);
        jobInputMatrices[0] = math.mul(robotFrameInv, eeToWorld);

        // Set Wrench
        jobInputWrench[0] = desiredWrench.Force.x;
        jobInputWrench[1] = desiredWrench.Force.y;
        jobInputWrench[2] = desiredWrench.Force.z;
        jobInputWrench[3] = desiredWrench.Torque.x;
        jobInputWrench[4] = desiredWrench.Torque.y;
        jobInputWrench[5] = desiredWrench.Torque.z;

        // Schedule
        var job = new CableSolverJob
        {
            numCables = numCables,
            eeToRobotFrame = jobInputMatrices,
            localAttachments = localAttachmentPoints,
            framePulleys = framePulleyPositions,
            targetWrench = jobInputWrench,
            beltCenterRef = beltCenterRef,
            
            tMin = minTension,
            tMax = maxTension,
            alpha = learningRate,
            lambda = regularization,
            iterations = maxIterations,
            
            tensions = currentTensions
        };

        currentJobHandle = job.Schedule();
    }

    // ============ OPTIMIZED SOLVER JOB ============

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    struct CableSolverJob : IJob
    {
        [ReadOnly] public int numCables;
        [ReadOnly] public NativeArray<double4x4> eeToRobotFrame;
        [ReadOnly] public NativeArray<double3> localAttachments;
        [ReadOnly] public NativeArray<double3> framePulleys;
        [ReadOnly] public NativeArray<double> targetWrench;
        [ReadOnly] public NativeArray<double3> beltCenterRef;

        [ReadOnly] public double tMin;
        [ReadOnly] public double tMax;
        [ReadOnly] public double alpha; 
        [ReadOnly] public double lambda; 
        [ReadOnly] public int iterations;

        public NativeArray<double> tensions; // R/W

        public unsafe void Execute()
        {
            // Allocate S matrix on Stack (fastest memory possible)
            double* S = stackalloc double[6 * numCables];

            double4x4 T = eeToRobotFrame[0];
            double3 beltCenter = beltCenterRef[0];
            
            // 1. Build Structure Matrix
            for (int i = 0; i < numCables; i++)
            {
                double4 p4 = new double4(localAttachments[i], 1.0);
                double3 attachPos = math.mul(T, p4).xyz;
                double3 diff = framePulleys[i] - attachPos;
                double len = math.length(diff);
                double3 u = (len > 1e-6) ? diff / len : new double3(0,0,0);

                double4 r_local = new double4(localAttachments[i] - beltCenter, 0.0);
                double3 r = math.mul(T, r_local).xyz; 
                double3 tau = math.cross(r, u); 

                S[0 * numCables + i] = u.x;
                S[1 * numCables + i] = u.y;
                S[2 * numCables + i] = u.z;
                S[3 * numCables + i] = tau.x;
                S[4 * numCables + i] = tau.y;
                S[5 * numCables + i] = tau.z;
            }

            // 2. Projected Gradient Descent
            double w0 = targetWrench[0]; double w3 = targetWrench[3];
            double w1 = targetWrench[1]; double w4 = targetWrench[4];
            double w2 = targetWrench[2]; double w5 = targetWrench[5];

            for (int iter = 0; iter < iterations; iter++)
            {
                // Forward: f = S * t
                double f0=0, f1=0, f2=0, t0=0, t1=0, t2=0;
                for (int i = 0; i < numCables; i++)
                {
                    double t = tensions[i];
                    f0 += S[0 * numCables + i] * t;
                    f1 += S[1 * numCables + i] * t;
                    f2 += S[2 * numCables + i] * t;
                    t0 += S[3 * numCables + i] * t;
                    t1 += S[4 * numCables + i] * t;
                    t2 += S[5 * numCables + i] * t;
                }

                // Residual: e = f - w
                double e0 = f0 - w0; double e3 = t0 - w3;
                double e1 = f1 - w1; double e4 = t1 - w4;
                double e2 = f2 - w2; double e5 = t2 - w5;

                // Backprop + Update
                for (int i = 0; i < numCables; i++)
                {
                    // Gradient of data term: S^T * e
                    double grad = S[0 * numCables + i] * e0 +
                                  S[1 * numCables + i] * e1 +
                                  S[2 * numCables + i] * e2 +
                                  S[3 * numCables + i] * e3 +
                                  S[4 * numCables + i] * e4 +
                                  S[5 * numCables + i] * e5;
                    
                    // Add Regularization gradient (lambda * t)
                    grad += lambda * tensions[i];

                    double tNext = tensions[i] - (alpha * grad);
                    tensions[i] = math.clamp(tNext, tMin, tMax);
                }
            }
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
        }
    }
    
    // Helper accessors from original code
    public NativeArray<double3> GetPulleyPositionsNative() => framePulleyPositions;
    
    public Vector3[] GetPulleyPositionsAsVector3Array()
    {
        var result = new Vector3[numCables];
        for (int i = 0; i < numCables; i++)
            result[i] = new Vector3((float)framePulleyPositions[i].x, (float)framePulleyPositions[i].y, (float)framePulleyPositions[i].z);
        return result;
    }
}