// ForcePlateCalibrator.cs
using UnityEngine;                 // Only for Vector3 overload compatibility + Debug if needed
using Unity.Mathematics;
using System.Security.Cryptography.X509Certificates;

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

    /// <summary>
    /// The force plate reads out TOTAL FORCE EXERTED, which is negative GRF.
    /// toggling invert_force_output = true makes sure the output of forcePlateManager() is GRF.
    /// </summary>
    public const bool invert_force_output = true;

    /// <summary>Scale factor for points (mm -> m)</summary>
    private const double mmToM = 0.001;

    public ForcePlateCalibrator()
    {
        // ---- 1) Measured corners in O0 (meters) ----

        // Small manual corrections (meters)
        double3 back_left_correction   = new double3( 0.03, -0.03, -0.01);
        double3 back_right_correction  = new double3( 0.03,  0.03, -0.01);
        double3 front_left_correction  = new double3(-0.03, -0.03, -0.01);
        double3 front_right_correction = new double3(-0.03,  0.03, -0.01);

        // Raw measured corner positions (meters) + corrections
        double3 back_left_corner   = new double3( 0.5385, 0.5793, -0.9520) + back_left_correction;
        double3 back_right_corner  = new double3( 0.5234, 1.1350, -0.9463) + back_right_correction;
        double3 front_left_corner  = new double3(-0.3202, 0.5510, -0.9525) + front_left_correction;
        double3 front_right_corner = new double3(-0.3341, 1.1077, -0.9458) + front_right_correction;

        double3 x_O0_raw = (front_right_corner - front_left_corner)  + (back_right_corner - back_left_corner);
        double3 y_O0_raw = (front_left_corner - back_left_corner) + (front_right_corner - back_right_corner);

        // ---- 2) Model corners in O1 (millimeters) ----
        // Obtained via GS orthogonalization
        double3 dir_x_O0 = math.normalize(x_O0_raw);
        double3 dir_y_O0 = y_O0_raw - math.dot(y_O0_raw, dir_x_O0) * dir_x_O0;
        dir_y_O0 = math.normalize(dir_y_O0);
        double3 dir_z_O0 = math.cross(dir_x_O0, dir_y_O0);
        R_only = new double3x3(dir_x_O0, dir_y_O0, dir_z_O0);
        t_O0 = (back_left_corner + back_right_corner) * 0.5;
        
    }

    // ===================== Public projection APIs =====================

    /// <summary>
    /// Project a local point from O1 (in millimeters) to O0 (in meters).
    /// Math-only path: double3.
    /// </summary>
    public void ProjectPosition(in double3 pO1_mm, out double3 pO0_m)
    {
        // pO0 = t + R * (mmToM * pO1_mm)
        pO0_m = t_O0 + math.mul(R_only, mmToM * pO1_mm);
    }

    /// <summary>
    /// Convenience overload for existing call sites that still pass Vector3.
    /// </summary>
    public void ProjectPosition(in Vector3 pO1_mm, out double3 pO0_m)
    {
        ProjectPosition(ToDouble3(pO1_mm), out pO0_m);
    }

    /// <summary>
    /// Project a local force vector from O1 to O0 (rotation only).
    /// Math-only path: double3.
    /// </summary>
    public void ProjectForce(in double3 fO1, out double3 fO0)
    {
        temp = math.mul(R_only, fO1);

        if (invert_force_output)
        {
            fO0 = -temp;
        } 
        else
        {
            fO0 = temp;
        }    
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
