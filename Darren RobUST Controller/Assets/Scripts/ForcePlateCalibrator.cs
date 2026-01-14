// ForcePlateCalibrator.cs
using UnityEngine;                 // Only for Vector3 overload compatibility + Debug if needed
using Unity.Mathematics;
using static Unity.Mathematics.math;

/// <summary>
/// Calibrates a rectangular plate's local frame O1 to global frame O0 via
/// rigid Procrustes (Davenport's q-method).
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

        // Order: FR, FL, BL, BR
        double3[] measuredO0 = new double3[4]
        {
            front_right_corner,
            front_left_corner,
            back_left_corner,
            back_right_corner
        };


        // ---- 2) Ideal model corners in O1 (meters) ----
        double widthMeters = 0.6;
        double heightMeters = 0.9;
        double hx = widthMeters * 0.5;

        double3[] modelO1 = new double3[4];
        modelO1[0] = new double3(+hx, heightMeters, 0.0); // FR
        modelO1[1] = new double3(-hx, heightMeters, 0.0); // FL
        modelO1[2] = new double3(-hx, 0.0,          0.0); // BL
        modelO1[3] = new double3(+hx, 0.0,          0.0); // BR

        // ---- 3) Centroids ----
        double3 cModel = Average(modelO1);
        double3 cMeas  = Average(measuredO0);

        // ---- 4) Centered sets ----
        double3[] Q = new double3[4];
        double3[] P = new double3[4];
        for (int i = 0; i < 4; i++)
        {
            Q[i] = modelO1[i] - cModel;    // O1 centered
            P[i] = measuredO0[i] - cMeas;  // O0 centered
        }

        // ---- 5) Cross-covariance B = sum_i Q_i * P_i^T ----
        // B is 3x3 where B[r,c] = sum Q[r] * P[c]
        double3x3 B = default;
        for (int i = 0; i < 4; i++)
        {
            // outer(Q, P): Q * P^T
            B.c0 += Q[i] * P[i].x; // column 0
            B.c1 += Q[i] * P[i].y; // column 1
            B.c2 += Q[i] * P[i].z; // column 2
        }

        // ---- 6) Davenport q-method: dominant eigenvector of K (4x4) ----
        double4x4 K = BuildDavenportK(B);
        double4 q = DominantEigenvector4x4(K, 60); // (w,x,y,z), normalized

        // ---- 7) Rotation from quaternion ----
        double3x3 R = RotationFromQuaternion(q);

        // Right-handedness guard (rare)
        if (det(R) < 0.0)
        {
            // Flip one column to fix reflection
            R.c2 = -R.c2;
        }

        // ---- 8) Translation t = cMeas - R * cModel ----
        double3 t = cMeas - mul(R, cModel);

        R_only = R;
        t_O0   = t;
    }

    // ===================== Public projection APIs =====================

    /// <summary>
    /// Project a local point from O1 (in millimeters) to O0 (in meters).
    /// Math-only path: double3.
    /// </summary>
    public void ProjectPosition(in double3 pO1_mm, out double3 pO0_m)
    {
        // pO0 = t + R * (mmToM * pO1_mm)
        pO0_m = t_O0 + mul(R_only, mmToM * pO1_mm);
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
        fO0 = mul(R_only, fO1);
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

    private static double3 Average(double3[] pts)
    {
        double3 s = double3.zero;
        for (int i = 0; i < pts.Length; i++) s += pts[i];
        return s / pts.Length;
    }

    /// <summary>
    /// Build Davenport K matrix from B (3x3 cross-covariance).
    /// K is 4x4 symmetric.
    /// </summary>
    private static double4x4 BuildDavenportK(in double3x3 B)
    {
        // B in column-major: B.c0=(B00,B10,B20), B.c1=(B01,B11,B21), B.c2=(B02,B12,B22)
        double Sxx = B.c0.x, Sxy = B.c1.x, Sxz = B.c2.x;
        double Syx = B.c0.y, Syy = B.c1.y, Syz = B.c2.y;
        double Szx = B.c0.z, Szy = B.c1.z, Szz = B.c2.z;

        double sigma = Sxx + Syy + Szz;
        double Zx = Syz - Szy;
        double Zy = Szx - Sxz;
        double Zz = Sxy - Syx;

        double S00 = Sxx + Sxx, S01 = Sxy + Syx, S02 = Sxz + Szx;
        double S10 = S01,       S11 = Syy + Syy, S12 = Syz + Szy;
        double S20 = S02,       S21 = S12,       S22 = Szz + Szz;

        // K as double4x4, column-major
        // rows/cols correspond to (w,x,y,z)
        double4 c0 = new double4(sigma, Zx,    Zy,    Zz);
        double4 c1 = new double4(Zx,    S00 - sigma, S01,  S02);
        double4 c2 = new double4(Zy,    S10,   S11 - sigma, S12);
        double4 c3 = new double4(Zz,    S20,   S21,  S22 - sigma);

        return new double4x4(c0, c1, c2, c3);
    }

    /// <summary>
    /// Power iteration for dominant eigenvector of symmetric 4x4.
    /// Returns normalized (w,x,y,z).
    /// </summary>
    private static double4 DominantEigenvector4x4(in double4x4 K, int iters)
    {
        double4 v = new double4(1.0, 0.0, 0.0, 0.0);
        for (int i = 0; i < iters; i++)
        {
            double4 Kv = mul(K, v);
            double n = length(Kv);
            if (n < 1e-14) break;
            v = Kv / n;
        }
        // Normalize defensively
        double nv = length(v);
        return (nv > 1e-14) ? (v / nv) : new double4(1.0, 0.0, 0.0, 0.0);
    }

    /// <summary>
    /// Quaternion (w,x,y,z) -> rotation matrix (double3x3).
    /// </summary>
    private static double3x3 RotationFromQuaternion(in double4 q)
    {
        double w = q.x, x = q.y, y = q.z, z = q.w;

        double ww = w*w, xx = x*x, yy = y*y, zz = z*z;

        // Row-major formula; then map into column-major double3x3
        double m00 = ww + xx - yy - zz;
        double m01 = 2.0*(x*y - w*z);
        double m02 = 2.0*(x*z + w*y);

        double m10 = 2.0*(x*y + w*z);
        double m11 = ww - xx + yy - zz;
        double m12 = 2.0*(y*z - w*x);

        double m20 = 2.0*(x*z - w*y);
        double m21 = 2.0*(y*z + w*x);
        double m22 = ww - xx - yy + zz;

        // double3x3 is column-major: c0=(m00,m10,m20), c1=(m01,m11,m21), c2=(m02,m12,m22)
        return new double3x3(
            new double3(m00, m10, m20),
            new double3(m01, m11, m21),
            new double3(m02, m12, m22)
        );
    }
}
