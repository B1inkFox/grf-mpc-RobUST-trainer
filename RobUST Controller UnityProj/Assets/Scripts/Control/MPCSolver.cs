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
    private double[] warmStart;                  // Solution buffer (reassigned by ALGLIB)
    
    private alglib.minqpstate qpState;
    private readonly int numCables;
    public readonly int horizon;
    public readonly int numVars;
    public readonly double dt;
    
    // ============ Cached State ============
    private double4x4 robotFramePose;
    private double4x4 endEffectorPose; 
    private double4x4 comPose;
    private RBState x0;                 // Current state: [p, Θ, v, ω]^T
    private double3x3 E_Theta;          // E(Θ): angular velocity to euler rates

    // MPC weights (public setters for gain scheduling)
    public double3 Q_pos { get; set; } = new double3(500.0, 500.0, 100.0);
    public double3 Q_Theta { get; set; } = new double3(0.001, 0.001, 0.001);
    public double3 Q_vel { get; set; } = new double3(50.0, 50.0, 50.0);
    public double3 Q_omega { get; set; } = new double3(0.001, 0.001, 0.001);
    public  readonly double alpha = 0.001;  // control effort weight
    private readonly double3x3 I_body; // Inertia tensor in body frame [kg·m²]
    private double3x3 I_world_inv;
    private RBState[] Xref;  // Reference trajectory over horizon
    private RBState[] freeResponseError;
    private double3[] B_v;
    private double3[] B_w;
    private double[,] M_pos; // Stores: Bv' * Qp * Bv + Bw' * E' * Qth * E * Bw
    private double[,] M_vel; // Stores: Bv' * Qv * Bv + Bw' * Qw * Bw
    
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
        Xref = new RBState[horizon];
        freeResponseError = new RBState[horizon];
        B_v = new double3[numCables];
        B_w = new double3[numCables];
        M_pos = new double[numCables, numCables];
        M_vel = new double[numCables, numCables];

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
    /// Expects poses to be pre-transformed into the robot frame.
    /// </summary>
    /// <param name="eePose_RF">End effector pose in robot frame.</param>
    /// <param name="comPose_RF">CoM pose in robot frame.</param>
    /// <param name="linearVelocity">CoM linear velocity in robot frame [m/s]</param>
    /// <param name="angularVelocity">CoM angular velocity in robot frame [rad/s]</param>
    /// <param name="netFPData">Net force plate data (already in robot frame)</param>
    public void UpdateState(double4x4 eePose_RF, double4x4 comPose_RF, 
                            in double3 linearVelocity, in double3 angularVelocity, 
                            in ForcePlateData netFPData)
    {
        // Store force plate data (already in robot frame)
        netGRF = netFPData.Force;
        netCoP = netFPData.CoP;

        endEffectorPose = eePose_RF;
        comPose = comPose_RF;
        
        double3x3 R_com = new double3x3(comPose.c0.xyz, comPose.c1.xyz, comPose.c2.xyz);
        x0 = new RBState(
            comPose.c3.xyz,
            RotationMatrixToEulerZYX(R_com),
            linearVelocity,
            angularVelocity
        );
        
        // Compute E(Θ) matrix for angular velocity to euler rates conversion
        double cos_psi = math.cos(x0.th.z);
        double cos_theta = math.cos(x0.th.y);
        double sin_psi = math.sin(x0.th.z);
        double sin_theta = math.sin(x0.th.y);
        E_Theta = new double3x3(
            cos_psi/cos_theta, sin_psi/cos_theta, 0.0,
            -sin_psi,  cos_psi, 0.0,
            cos_psi * sin_theta/cos_theta, sin_psi * sin_theta/cos_theta, 1.0
        );

        double3x3 I_body_inv = math.inverse(I_body);
        double3x3 R_curr = new double3x3(comPose.c0.xyz, comPose.c1.xyz, comPose.c2.xyz);
        I_world_inv = math.mul(math.mul(R_curr, I_body_inv), math.transpose(R_curr));

        // Build Bv and Bw matrices
        for (int i = 0; i < numCables; i++)
        {
            double3 attachLocal = robot.LocalAttachmentPoints[i];
            double3 attachCurrent = math.mul(endEffectorPose, new double4(attachLocal, 1.0)).xyz;
            
            // Compute Direction (n_i) and Moment Arm (r_i)
            double3 n_i = math.normalize(robot.FramePulleyPositions[i] - attachCurrent);
            double3 r_i = attachCurrent - x0.p; // Vector from CoM to attachment

            B_v[i] = n_i * (dt / robot.UserMass); 

            double3 torque_unit = math.cross(r_i, n_i);
            B_w[i] = math.mul(I_world_inv, torque_unit) * dt;
        }
    }


    private void BuildQuadraticCost()
    {
        // Recall M_pos: Bv'Qp Bv + Bw'E'Qth E Bw
        // Recall M_vel: Bv'Qv Bv + Bw'Qw Bw
        
        // 1. Precompute the effective Angular Weight Matrix in body frame: W = E^T * Q_Theta * E
        double3x3 ET_Qtheta_E = new double3x3();
        ET_Qtheta_E.c0 = math.mul(math.transpose(E_Theta), Q_Theta * E_Theta.c0);
        ET_Qtheta_E.c1 = math.mul(math.transpose(E_Theta), Q_Theta * E_Theta.c1);
        ET_Qtheta_E.c2 = math.mul(math.transpose(E_Theta), Q_Theta * E_Theta.c2);

        // 2. Fill Atomic M_pos M_vel Matrices (Nc x Nc)
        for (int r = 0; r < numCables; r++)
        {
            for (int c = 0; c < numCables; c++)
            {               
                // ============ Position & Orientation Terms ============
                // Linear Position Term: B_v^T * Q_pos * B_v
                double BvT_Qpos_Bv = math.dot(B_v[r], Q_pos * B_v[c]);
                
                // We first compute the right-hand side vector: (E^T Q E) * B_w[c]
                double3 ETQthetaE_Bw_c = math.mul(ET_Qtheta_E, B_w[c]);
                double BwT_ETQthetaE_Bw = math.dot(B_w[r], ETQthetaE_Bw_c);

                // Combine into the Position Block
                M_pos[r, c] = BvT_Qpos_Bv + BwT_ETQthetaE_Bw;


                // ============ Velocity & Angular Velocity Terms ============
                // Linear Velocity Term: B_v^T * Q_vel * B_v
                double BvT_Qvel_Bv = math.dot(B_v[r], Q_vel * B_v[c]);
                
                // Angular Velocity Term: B_w^T * Q_omega * B_w
                double BwT_Qomega_Bw = math.dot(B_w[r], Q_omega * B_w[c]);

                // Combine into the Velocity Block
                M_vel[r, c] = BvT_Qvel_Bv + BwT_Qomega_Bw;
            }
        }

        // 2. Fill Global Hessian H (Blockwise)
        // ---------------------------------------------------------        
        double dtSq = dt * dt;

        // Iterate Upper Triangle of Blocks
        for (int i = 0; i < horizon; i++)
        {
            for (int j = i; j < horizon; j++)
            {
                double coeff_vel = (double)(horizon - j);

                // Coeff for Position Terms: Sum of Squares series
                // S is the number of steps remaining after the later input j
                // Formula: Sum_{k=0 to S} [k^2 + (j-i)k]
                // SumSq = S(S+1)(2S+1)/6
                // SumLin = S(S+1)/2
                long S = horizon - 1 - j; 
                double sum_sq = (double)(S * (S + 1) * (2 * S + 1)) / 6.0;
                double sum_lin = (double)(S * (S + 1)) / 2.0;
                double coeff_pos = sum_sq + (j - i) * sum_lin;
                coeff_pos *= dtSq;

                int r_offset = i * numCables;
                int c_offset = j * numCables;

                for (int r = 0; r < numCables; r++)
                {
                    for (int c = 0; c < numCables; c++)
                    {
                        double val = 2.0 * (coeff_pos * M_pos[r, c] + coeff_vel * M_vel[r, c]);

                        // Assign to H (Upper Triangle)
                        H_matrix[r_offset + r, c_offset + c] = val;

                        // Symmetry (Lower Triangle)
                        if (i != j)
                            H_matrix[c_offset + c, r_offset + r] = val;
                    }
                }
            }
            
            // Add Regularization (R_bar) to Diagonal elements of this block
            // (Only for i==j blocks)
            int diag_offset = i * numCables;
            for (int r = 0; r < numCables; r++)
            {
                H_matrix[diag_offset + r, diag_offset + r] += 2.0 * alpha;
            }
        }
    }

    private void BuildLinearCost()
    {
        // 0. Setup Dynamics Terms (Ad, Bd, d) derived from x0
        // ---------------------------------------------------------        
        // Linear acceleration part: (g + F_grf / m) * dt
        double3 d_vel = (g_vec + (netGRF / robot.UserMass)) * dt;
        
        // Angular acceleration part: I_inv * ( Torque_grf ) * dt
        double3 r_cop = netCoP - x0.p; 
        double3 tau_grf = math.cross(r_cop, netGRF);
        double3 d_ang = math.mul(I_world_inv, tau_grf) * dt;

        // 1. Forward Pass: "Free Response" Simulation
        // ---------------------------------------------------------
        // x_{k+1} = A_d * x_k + d
        
        RBState x_pred = x0; // Start at current state

        for (int k = 0; k < horizon; k++)
        {
            // --- Apply Discrete Dynamics (Ad * x + d) ---
            x_pred.p += x_pred.v * dt;
            x_pred.th += math.mul(E_Theta, x_pred.w) * dt;
            x_pred.v += d_vel;
            x_pred.w += d_ang;

            // --- Compute Error against Reference ---
            RBState error;
            error.p = x_pred.p - Xref[k].p;
            error.th = x_pred.th - Xref[k].th; // TODO: Handle angle wrapping if necessary
            error.v = x_pred.v - Xref[k].v;
            error.w = x_pred.w - Xref[k].w;

            // Store (Q * (Ax + d - xref)) for the backward pass
            freeResponseError[k].p = error.p * Q_pos;
            freeResponseError[k].th = error.th * Q_Theta;
            freeResponseError[k].v = error.v * Q_vel;
            freeResponseError[k].w = error.w * Q_omega;
        }

        // 2. Backward Pass: Adjoint Calculation (Co-state)
        // ---------------------------------------------------------
        // Calculates gradient of cost w.r.t inputs: g = 2 * B^T * p
        // p_k = Q*e_{k+1} + A_d^T * p_{k+1}
        RBState p = new RBState(); // "Co-state"
        for (int k = horizon - 1; k >= 0; k--)
        {
            // Accumulate error
            p.p += freeResponseError[k].p;
            p.th += freeResponseError[k].th;
            p.v += freeResponseError[k].v;
            p.w += freeResponseError[k].w;

            // --- Linear Cost Gradient (g_k) ---
            // g = 2 * B^T * p
            int stepOffset = k * numCables;
            for (int i = 0; i < numCables; i++)
            {
                double val_v = math.dot(B_v[i], p.v);
                double val_w = math.dot(B_w[i], p.w);

                g_qp[stepOffset + i] = 2.0 * (val_v + val_w);
            }

            // --- Propagate Co-state (p = Ad^T * p) ---
            p.v += p.p * dt;
            p.w += math.mul(math.transpose(E_Theta), p.th) * dt;
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
        alglib.minqpresults(qpState, out warmStart, out alglib.minqpreport report);
        
        // 4. Check solver status and extract first timestep tensions
        if (report.terminationtype > 0)
        {
            Buffer.BlockCopy(warmStart, 0, tensions, 0, numCables * sizeof(double));
        } else {
            Debug.LogWarning($"MPC QP solver failed. Termination type: {report.terminationtype}");
        }
        // else: keep previous tensions (fail-safe)
        
        return tensions;
    }

    /// <summary>
    /// Updates the local reference trajectory from a global plan.
    /// Handles extraction of the relevant window and padding with the final goal.
    /// </summary>
    /// <param name="globalPlan">The complete trajectory reference.</param>
    /// <param name="startIndex">Current time index in the global plan.</param>
    public void UpdateReferenceTrajectory(ReadOnlySpan<RBState> globalPlan, int startIndex)
    {
        if (globalPlan.IsEmpty) return;

        int availableSteps = math.max(0, globalPlan.Length - startIndex);
        int copyCount = math.min(horizon, availableSteps);

        if (copyCount > 0) globalPlan.Slice(startIndex, copyCount).CopyTo(Xref.AsSpan(0, copyCount));

        if (copyCount < horizon)
        {
            RBState finalGoal = globalPlan[globalPlan.Length - 1];
            Xref.AsSpan(copyCount).Fill(finalGoal);
        }
    }

    // ============ Visualization Helpers ============


    public void ComputeOptimalTrajectory(Span<RBState> results)
    {
        if (warmStart == null || warmStart.Length < numVars) return;
        ComputeTrajectory(results, useControl: true);
    }
    public void ComputeFreeTrajectory(Span<RBState> results)
    {
        ComputeTrajectory(results, useControl: false);
    }
    private void ComputeTrajectory(Span<RBState> results, bool useControl)
    {
        // Re-calculate constant disturbance terms locally 
        // (Must match BuildLinearCost logic exactly)
        double3 d_vel = dt * (g_vec + (netGRF / robot.UserMass));
        
        double3 r_cop = netCoP - x0.p; 
        double3 tau_grf = math.cross(r_cop, netGRF);
        double3 d_ang = dt * math.mul(I_world_inv, tau_grf);

        RBState x_k = x0;
        int steps = math.min(results.Length, horizon);

        for (int k = 0; k < steps; k++)
        {
            x_k.p += x_k.v * dt;
            x_k.th += math.mul(E_Theta, x_k.w) * dt;
            x_k.v += d_vel;
            x_k.w += d_ang;

            if (useControl)
            {
                int u_offset = k * numCables;
                for (int i = 0; i < numCables; i++)
                {
                    double u = warmStart[u_offset + i];
                    x_k.v += B_v[i] * u;
                    x_k.w += B_w[i] * u;
                }
            }
            
            // Write to buffer
            results[k] = x_k;
        }
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
    
}