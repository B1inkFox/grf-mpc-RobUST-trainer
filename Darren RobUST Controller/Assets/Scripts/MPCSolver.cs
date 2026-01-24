using Unity.Mathematics;
using UnityEngine;
using System;

/// <summary>
/// MPC-based controller that directly computes cable tensions.
/// Uses ALGLIB QuickQP with box constraints for real-time performance.
/// </summary>
public class MPCSolver : BaseController<double[]>
{
    private readonly RobUSTDescription robot;
    // ============ QP Solver Parameters ============
    private static readonly double MinTension = 10.0;    // [N]
    private static readonly double MaxTension = 200.0;  // [N]
    private static readonly double3 g_vec = new double3(0.0, 0.0, -9.81); // gravity vector [m/s²]
    
    // ============ Pre-allocated QP structures (zero runtime allocation) ============
    private readonly double[] tensions;          // Output buffer
    private readonly double[,] H_matrix;    // H matrix
    private readonly double[] g_qp;        // g_qp vector
    private readonly double[] lowerBounds;       // Box constraint lower
    private readonly double[] upperBounds;       // Box constraint upper
    private readonly double[] warmStart;         // Previous solution
    
    private alglib.minqpstate qpState;
    private readonly int numCables;
    public readonly int horizon;
    public readonly int numVars;
    public readonly double dt;
    
    // ============ Cached State (x0 = [p, Θ, v, ω]^T (12-dim)) ============
    // All poses/velocities expressed relative to robot frame origin
    private double4x4 robotFramePose;
    private double4x4 endEffectorPose; 
    private double4x4 comPose; 
    private double3 comPosition;        // p: CoM position [m]
    private double3 comEulerAnglesZYX;  // Θ: ZYX Euler angles [rad]
    private double3 comLinearVelocity;  // v: linear velocity [m/s]
    private double3 comAngularVelocity; // ω: angular velocity [rad/s]
    private double3x3 E_Theta;        // E(Θ) angular velocity to euler rates conversion matrix

    // Cached MPC parameters
    private readonly double3 Kp;
    private readonly double3 KTheta;
    private readonly double3 Kv;
    private readonly double3 Kw;
    private double alpha = 0.00001;  // control effort weight
    private readonly double3x3 I_body; // Inertia tensor in body frame [kg·m²]
    private double3x3 I_world_inv;
    private double3[] Xref;  // Reference trajectory over horizon
    
    // Force plate data (already in robot frame)
    private double3 netGRF;             // Total ground reaction force [N]
    private double3 netCoP;             // Global center of pressure [m]
    
    /// <summary>
    /// Creates MPC solver using robot description for all parameters.
    /// </summary>
    /// <param name="robotDescription">Robot description (owned by RobotController)</param>
    /// <param name="timestep">MPC timestep [s]</param>
    /// <param name="predictionHorizon">Number of timesteps to predict</param>
    public MPCSolver(RobUSTDescription robotDescription, double timestep, int predictionHorizon)
    {
        robot = robotDescription ?? throw new ArgumentNullException(nameof(robotDescription));
        dt = timestep;
        horizon = predictionHorizon;
        numCables = robot.NumCables;
        numVars = numCables * horizon;
        
        // Single allocation at construction
        tensions = new double[numCables];
        H_matrix = new double[numVars, numVars];
        g_qp = new double[numVars];
        lowerBounds = new double[numVars];
        upperBounds = new double[numVars];
        warmStart = new double[numVars];
        I_body = new double3x3(
            robot.UserMass / 12.0 * (3*math.pow(robot.UserShoulderWidth / 2.0, 2) + math.pow(robot.UserHeight, 2)), 0.0, 0.0,
            0.0, robot.UserMass / 12.0 * (3*math.pow(robot.UserShoulderWidth / 2.0, 2) + math.pow(robot.UserHeight, 2)), 0.0,
            0.0, 0.0, robot.UserMass / 2.0 * math.pow(robot.UserShoulderWidth / 2.0, 2)
        );
        Xref = new double3[horizon * 4];

        // Initialize box constraints (constant, set once)
        for (int i = 0; i < numVars; i++)
        {
            lowerBounds[i] = MinTension;
            upperBounds[i] = MaxTension;
            warmStart[i] = (MinTension + MaxTension) * 0.5;  // Start at midpoint
        }
        
        // Initialize solver once
        alglib.minqpcreate(numVars, out qpState);
        alglib.minqpsetalgoquickqp(qpState, 1e-6, 0.0, 0.0, 30, true);
        alglib.minqpsetbc(qpState, lowerBounds, upperBounds);
    }

    
    /// <summary>
    /// Updates cached state before solving. Call this every control loop iteration.
    /// Converts raw TrackerData to robot frame and computes velocities via finite differencing.
    /// </summary>
    public void UpdateState(in TrackerData rawRobotFrame, in TrackerData rawEndEffector, 
                            in TrackerData rawComTracker, in TrackerData rawComTrackerPrev,
                            in ForcePlateData netFPData)
    {
        // Store force plate data (already in robot frame)
        netGRF = netFPData.Force;
        netCoP = netFPData.CenterOfPressure;

        // Convert to double4x4 and compute robot frame inverse
        robotFramePose = ToDouble4x4(rawRobotFrame.PoseMatrix);
        double4x4 robotFrameInv = math.fastinverse(robotFramePose);
        
        double4x4 rawEePose = ToDouble4x4(rawEndEffector.PoseMatrix);
        double4x4 rawComPose = ToDouble4x4(rawComTracker.PoseMatrix);
        double4x4 rawComPosePrev = ToDouble4x4(rawComTrackerPrev.PoseMatrix);
        
        // Transform poses to robot frame: T = T_frame^-1 * T_raw
        endEffectorPose = math.mul(robotFrameInv, rawEePose);
        comPose = math.mul(robotFrameInv, rawComPose);
        double4x4 comPosePrev = math.mul(robotFrameInv, rawComPosePrev);
        
        // Extract CoM position (p)
        comPosition = comPose.c3.xyz;
        double3 comPositionPrev = comPosePrev.c3.xyz;
        
        // Extract rotation matrix and compute ZYX Euler angles (Θ)
        double3x3 R_com = new double3x3(comPose);
        comEulerAnglesZYX = RotationMatrixToEulerZYX(R_com);
        
        // Finite difference for linear velocity: v = (p_curr - p_prev) / dt
        comLinearVelocity = (comPosition - comPositionPrev) / dt;
        
        // Finite difference for angular velocity
        double3x3 R_comPrev = new double3x3(comPosePrev);
        double3x3 R_diff = math.mul(R_com, math.transpose(R_comPrev));
        
        // Extract ω from skew-symmetric part: ω = [R32-R23, R13-R31, R21-R12] / (2*dt)
        comAngularVelocity = new double3(
            R_diff.c1.z - R_diff.c2.y,
            R_diff.c2.x - R_diff.c0.z,
            R_diff.c0.y - R_diff.c1.x
        ) / (2.0 * dt);
        
        // Compute E(Θ) matrix for angular velocity to euler rates conversion
        double cos_psi = math.cos(comEulerAnglesZYX.z);
        double cos_theta = math.cos(comEulerAnglesZYX.y);
        double sin_psi = math.sin(comEulerAnglesZYX.z);
        double sin_theta = math.sin(comEulerAnglesZYX.y);
        E_Theta = new double3x3(
            cos_psi/cos_theta, sin_psi/cos_theta, 0.0,
            -sin_psi,  cos_psi, 0.0,
            cos_psi * sin_theta/cos_theta, sin_psi * sin_theta/cos_theta, 1.0
        );

        double3x3 I_body_inv = math.inverse(I_body);
        double3x3 R_curr = new double3x3(comPose.c0.xyz, comPose.c1.xyz, comPose.c2.xyz);
        I_world_inv = math.mul(math.mul(R_curr, I_body_inv), math.transpose(R_curr));
    }

    private void BuildQuadraticCost()
    {
        

    }

    private void BuildLinearCost()
    {
        double3 p_curr = comPosition;
        double3 v_curr = comLinearVelocity;
        double3 th_curr = comEulerAnglesZYX;
        double3 w_curr = comAngularVelocity;

        for (int k = 0; k < horizon; k++)
        {
            p_curr += v_curr * dt;
            th_curr += math.mul(E_Theta, w_curr) * dt;
        }
    }
    
    
    /// <summary>
    /// Computes optimal cable tensions for current state.
    /// Returns pre-allocated array - do NOT hold reference across frames.
    /// </summary>
    public override double[] computeNextControl()
    {
        BuildQuadraticCost();
        BuildLinearCost();
        
        // 2. Update solver with new cost terms
        alglib.minqpsetquadraticterm(qpState, H_matrix, false);
        alglib.minqpsetlinearterm(qpState, g_qp);
        alglib.minqpsetstartingpoint(qpState, warmStart);
        
        // 3. Solve
        alglib.minqpoptimize(qpState);
        alglib.minqpresults(qpState, out double[] solution, out alglib.minqpreport report);
        
        // 4. Check solver status and extract first timestep tensions
        if (report.terminationtype > 0)
        {
            // Success - extract first timestep (indices 0 to numCables-1)
            Buffer.BlockCopy(solution, 0, tensions, 0, numCables * sizeof(double));
            
            // Update warm-start with full solution for next iteration
            Buffer.BlockCopy(solution, 0, warmStart, 0, numVars * sizeof(double));
        } else {
            Debug.LogWarning($"MPC QP solver failed. Termination type: {report.terminationtype}"); // will remove once validated
        }
        // else: keep previous tensions (fail-safe)
        
        return tensions;
    }

    /// <summary>
    /// Provides access to reference trajectory buffer for external population.
    /// </summary>
    public Span<double3> GetXref()
    {
        return Xref.AsSpan();
    }

    private static double3 RotationMatrixToEulerZYX(in double3x3 R)
    {
        double sy = -R.c0.z;  // -R13
        double pitch, roll, yaw;
        
        if (math.abs(sy) < 0.99999)
        {
            pitch = math.asin(sy);
            roll = math.atan2(R.c1.z, R.c2.z);   // atan2(R23, R33)
            yaw = math.atan2(R.c0.y, R.c0.x);    // atan2(R12, R11)
        }
        else
        {
            // Gimbal lock: pitch ≈ ±90°
            pitch = sy > 0 ? math.PI / 2.0 : -math.PI / 2.0;
            roll = 0.0;
            yaw = math.atan2(-R.c1.x, R.c1.y);   // atan2(-R21, R22)
        }
        
        return new double3(roll, pitch, yaw);  // [Θx, Θy, Θz]
    }
    
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