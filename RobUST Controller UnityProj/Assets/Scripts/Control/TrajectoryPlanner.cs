using Unity.Mathematics;
using System;

public class TrajectoryPlanner
{
    private RBState[] Xref_global;
    
    public TrajectoryPlanner()
    {
        // Initialize Global Reference Trajectory
        Xref_global = new RBState[2000]; // 20 seconds buffer
        // stable standing
        RBState staticPoint = new RBState(
            new double3(0.2, 0.75, -0.1), 
            new double3(0, 0, math.PI/2), 
            new double3(0, 0, 0), 
            new double3(0, 0, 0)
        );
        for (int i = 0; i < Xref_global.Length; i++) Xref_global[i] = staticPoint;
    }

    /// <summary>
    /// Safely fills the destination span with the lookahead trajectory.
    /// </summary>
    public void FillHorizon(long trajectoryIndex, Span<RBState> destination)
    {
        int startIndex = (int)trajectoryIndex;
        int sourceLen = Xref_global.Length;
        int destinationLen = destination.Length;

        if (startIndex >= sourceLen)
        {
            destination.Fill(Xref_global[sourceLen - 1]);
            return;
        }

        int copyCount = math.min(destinationLen, sourceLen - startIndex);
        Xref_global.AsSpan(startIndex, copyCount).CopyTo(destination);

        if (copyCount < destinationLen)
            destination.Slice(copyCount).Fill(Xref_global[sourceLen - 1]);
    }

    /// <summary>
    /// Computes the corresponding End-Effector reference state from a CoM reference state.
    /// Uses the current body-frame offset r_local = R_curr^T * (p_ee - p_com)
    /// and rotates it by the reference orientation to support full rigid body kinematics.
    /// </summary>
    public static RBState ComputeEERefFromCoMRef(RBState comRefState, double4x4 curr_comPose, double4x4 curr_eePose)
    {
        // 1. Calculate constant offset in body frame: r_local = R_com^T * (p_ee - p_com)
        double3 p_diff_world = curr_eePose.c3.xyz - curr_comPose.c3.xyz;
        double3x3 R_curr = new double3x3(curr_comPose.c0.xyz, curr_comPose.c1.xyz, curr_comPose.c2.xyz);
        double3 r_local = math.mul(math.transpose(R_curr), p_diff_world);

        // 2. Compute Rotation Matrix for Reference State (Euler ZYX)
        double3 th = comRefState.th; // (x:phi, y:theta, z:psi)
        
        // Build R_ref = Rz(psi) * Ry(theta) * Rx(phi) manually (Column-Major)
        double cz = math.cos(th.z), sz = math.sin(th.z);
        double cy = math.cos(th.y), sy = math.sin(th.y);
        double cx = math.cos(th.x), sx = math.sin(th.x);
        double3x3 R_z = new double3x3(new double3(cz, sz, 0), new double3(-sz, cz, 0), new double3(0, 0, 1));
        double3x3 R_y = new double3x3(new double3(cy, 0, -sy), new double3(0, 1, 0), new double3(sy, 0, cy));
        double3x3 R_x = new double3x3(new double3(1, 0, 0), new double3(0, cx, sx), new double3(0, -sx, cx));
        double3x3 R_ref = math.mul(R_z, math.mul(R_y, R_x));

        // 3. Compute EE Reference terms
        double3 r_ref_global = math.mul(R_ref, r_local);
        
        return new RBState(
            comRefState.p + r_ref_global,                          // p_ee_ref
            comRefState.th,                                        // Orientation (Shared)
            comRefState.v + math.cross(comRefState.w, r_ref_global), // v_ee_ref = v_com + w x r
            comRefState.w                                          // Angular Velocity (Shared)
        );
    }
}
