// ForcePlateCalibrator.cs
using UnityEngine;                 // Only for Vector3 overload compatibility + Debug if needed
using Unity.Mathematics;

/// <summary>
/// Calibrates a rectangular plate's local frame O1 to global frame O0
///
/// Uses Unity.Mathematics for all non-visualization math:
/// - Rotation stored as double3x3
/// - Translation stored as double3
/// - Point transform uses explicit scale (mm->m), then rotate, then translate
///
/// Corner/model ordering (must correspond 1:1):
/// measured[0]=FR, measured[1]=FL, measured[2]=BL, measured[3]=BR
/// model[0]=( +W/2, +H, 0), model[1]=( -W/2, +H, 0),
/// model[2]=( -W/2,  0, 0), model[3]=( +W/2,  0, 0)
/// </summary>
public sealed class ForcePlateCalibrator
{
    // ---- Public read-only results ----

    /// <summary>Rotation matrix O1 -> O0</summary>
    public readonly double3x3 R_only;

    /// <summary>Translation of O1 origin in O0</summary>
    public readonly double3 t_O0;


    public ForcePlateCalibrator(RobUSTDescription robot)
    {
        double3 x_O0_raw = (robot.FP_FrontRight - robot.FP_FrontLeft)  + (robot.FP_BackRight - robot.FP_BackLeft);
        double3 y_O0_raw = (robot.FP_FrontLeft - robot.FP_BackLeft) + (robot.FP_FrontRight - robot.FP_BackRight);

        // ---- 2) Model corners in O1 (millimeters) ----
        // Obtained via GS orthogonalization
        double3 dir_x_O0 = math.normalize(x_O0_raw);
        double3 dir_y_O0 = y_O0_raw - math.dot(y_O0_raw, dir_x_O0) * dir_x_O0;
        dir_y_O0 = math.normalize(dir_y_O0);
        double3 dir_z_O0 = math.cross(dir_x_O0, dir_y_O0);
        R_only = new double3x3(dir_x_O0, dir_y_O0, dir_z_O0);
        t_O0 = (robot.FP_BackLeft + robot.FP_BackRight) * 0.5;
        
    }

    // ===================== Public projection APIs =====================

    /// <summary>
    /// Project a local point from O1 to O0.
    /// Math-only path: double3.
    /// </summary>
    public void ProjectPosition(in double3 pO1_m, out double3 pO0_m)
    {
        // pO0 = t + R * pO1_m
        pO0_m = t_O0 + math.mul(R_only, pO1_m);
    }

    /// <summary>
    /// Convenience overload for existing call sites that still pass Vector3.
    /// </summary>
    public void ProjectPosition(in Vector3 pO1_m, out double3 pO0_m)
    {
        ProjectPosition(ToDouble3(pO1_m), out pO0_m);
    }

    /// <summary>
    /// Project a local force vector from O1 to O0 (rotation only).
    /// Math-only path: double3.
    /// </summary>
    public void ProjectForce(in double3 fO1, out double3 fO0)
    {
        fO0 = -math.mul(R_only, fO1);
    }

    /// <summary>
    /// Convenience overload for existing call sites that still pass Vector3.
    /// </summary>
    public void ProjectForce(in Vector3 fO1, out double3 fO0)
    {
        ProjectForce(ToDouble3(fO1), out fO0);
    }

    // ===================== Helpers =====================

    private static double3 ToDouble3(in Vector3 v) => new double3(v.x, v.y, v.z);

}
