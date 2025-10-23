using UnityEngine;

/// <summary>
/// Calibrates a rectangular plate's local frame O1 to global frame O0 via
/// rigid Procrustes (Davenport's q-method). All heavy math runs in the constructor.
/// 
/// Usage:
///   var calib = new ForcePlateCalibrator();
///   Vector3 cop_O0   = calib.ProjectPosition(cop_O1_mm); // mm -> m with TRS
///   Vector3 force_O0 = calib.ProjectForce(force_O1);      // rotate only
/// 
/// Corner/model ordering (must correspond 1:1):
/// measured[0]=FR, measured[1]=FL, measured[2]=BL, measured[3]=BR
/// model[0]=( +W/2, +H, 0), model[1]=( -W/2, +H, 0),
/// model[2]=( -W/2,  0, 0), model[3]=( +W/2,  0, 0)
/// </summary>
public class ForcePlateCalibrator
{
    // Public read-only results (if you want to inspect)
    public readonly Matrix4x4 R_only;   // rotation O1->O0
    public readonly Vector3   t_O0;     // translation of O1 origin in O0
    public readonly Matrix4x4 T_point;  // T * R * S(mm->m) for points

    /// <summary>
    /// Constructor performs the full calibration.
    /// </summary>
    public ForcePlateCalibrator()
    {
        float mmToM = 0.001f;

        Vector3 back_left_correction = new Vector3(0.03f, -0.03f, -0.01f);
        Vector3 back_right_correction = new Vector3(0.03f, 0.03f, -0.01f);
        Vector3 front_left_correction = new Vector3(-0.03f, -0.03f, -0.01f);
        Vector3 front_right_correction = new Vector3(-0.03f, 0.03f, -0.01f);

        Vector3 back_left_corner = new Vector3(0.5385f, 0.5793f, -0.9520f) + back_left_correction;
        Vector3 back_right_corner = new Vector3(0.5234f, 1.1350f, -0.9463f) + back_right_correction;
        Vector3 front_left_corner = new Vector3(-0.3202f, 0.5510f, -0.9525f) + front_left_correction;
        Vector3 front_right_corner = new Vector3(-0.3341f, 1.1077f, -0.9458f) + front_right_correction;
        
        // --- 1) Measured corners in O0 (meters), in the specified order ---
        Vector3[] measuredO0 = new Vector3[4] {
            front_right_corner, front_left_corner, back_left_corner, back_right_corner
        };

        // --- 2) Ideal model corners in O1 (meters), origin on bottom edge center, z=0 ---
        float widthMeters = 0.6f;
        float heightMeters = 0.9f;
        float hx = widthMeters * 0.5f;
        Vector3[] modelO1 = new Vector3[4];
        modelO1[0] = new Vector3(+hx, heightMeters, 0f); // FR
        modelO1[1] = new Vector3(-hx, heightMeters, 0f); // FL
        modelO1[2] = new Vector3(-hx, 0f,          0f);  // BL
        modelO1[3] = new Vector3(+hx, 0f,          0f);  // BR

        // --- 3) Centroids ---
        Vector3 cModel = Average(modelO1);
        Vector3 cMeas  = Average(measuredO0);

        // --- 4) Centered sets ---
        Vector3[] Q = new Vector3[4];
        Vector3[] P = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
            Q[i] = modelO1[i] - cModel;   // O1 centered
            P[i] = measuredO0[i] - cMeas; // O0 centered
        }

        // --- 5) Cross-covariance B = sum_i Q_i * P_i^T ---
        float[,] B = new float[3,3];
        for (int i = 0; i < 4; i++)
        {
            B[0,0] += Q[i].x * P[i].x;  B[0,1] += Q[i].x * P[i].y;  B[0,2] += Q[i].x * P[i].z;
            B[1,0] += Q[i].y * P[i].x;  B[1,1] += Q[i].y * P[i].y;  B[1,2] += Q[i].y * P[i].z;
            B[2,0] += Q[i].z * P[i].x;  B[2,1] += Q[i].z * P[i].y;  B[2,2] += Q[i].z * P[i].z;
        }

        // --- 6) Davenport’s q-method: dominant eigenvector of K (Horn 1987) ---
        double[,] K = BuildDavenportK(B);
        Vector4 q = DominantEigenvector4x4(K, 50).normalized; // (w,x,y,z)

        // --- 7) Rotation matrix O1->O0 from quaternion ---
        Matrix4x4 R = MatrixFromQuaternion(q);

        // Right-handedness fix (very rare)
        if (Determinant3x3(R) < 0f)
        {
            var col2 = R.GetColumn(2);
            R.SetColumn(2, -col2);
        }

        // --- 8) Translation t = cMeas - R * cModel ---
        Vector3 t = cMeas - R.MultiplyVector(cModel);

        // --- 9) Build point transform: T * R * S(mm->m) ---
        Matrix4x4 T = Matrix4x4.Translate(t);
        Matrix4x4 S = Matrix4x4.Scale(new Vector3(mmToM, mmToM, mmToM));

        // Expose results
        R_only = R;
        t_O0   = t;
        T_point = T * R * S;
    }

    /// <summary>
    /// Project a local point (e.g., CoP) from O1 (in millimeters) to O0 (in meters).
    /// </summary>
    public Vector3 ProjectPosition(Vector3 pO1_mm)
    {
        return T_point.MultiplyPoint3x4(pO1_mm);
    }

    /// <summary>
    /// Project a local vector (e.g., Force) from O1 to O0 (rotation only).
    /// </summary>
    public Vector3 ProjectForce(Vector3 fO1)
    {
        return R_only.MultiplyVector(fO1);
    }

    // ----------------- Helpers -----------------

    private static Vector3 Average(Vector3[] pts)
    {
        Vector3 s = Vector3.zero;
        for (int i = 0; i < pts.Length; i++) s += pts[i];
        return s / pts.Length;
    }

    private static double[,] BuildDavenportK(float[,] B)
    {
        double Sxx = B[0,0], Sxy = B[0,1], Sxz = B[0,2];
        double Syx = B[1,0], Syy = B[1,1], Syz = B[1,2];
        double Szx = B[2,0], Szy = B[2,1], Szz = B[2,2];

        double sigma = Sxx + Syy + Szz;
        double Zx = Syz - Szy;
        double Zy = Szx - Sxz;
        double Zz = Sxy - Syx;

        double S00 = Sxx + Sxx, S01 = Sxy + Syx, S02 = Sxz + Szx;
        double S10 = S01,       S11 = Syy + Syy, S12 = Syz + Szy;
        double S20 = S02,       S21 = S12,       S22 = Szz + Szz;

        double[,] K = new double[4,4];
        K[0,0] = sigma; K[0,1] = Zx;           K[0,2] = Zy;           K[0,3] = Zz;
        K[1,0] = Zx;    K[1,1] = S00 - sigma;  K[1,2] = S01;          K[1,3] = S02;
        K[2,0] = Zy;    K[2,1] = S10;          K[2,2] = S11 - sigma;  K[2,3] = S12;
        K[3,0] = Zz;    K[3,1] = S20;          K[3,2] = S21;          K[3,3] = S22 - sigma;
        return K;
    }

    private static Vector4 DominantEigenvector4x4(double[,] K, int iters)
    {
        Vector4 v = new Vector4(1f, 0f, 0f, 0f);
        for (int i = 0; i < iters; i++)
        {
            Vector4 Kv = Mul4x4(K, v);
            float n = Kv.magnitude;
            if (n < 1e-12f) break;
            v = Kv / n;
        }
        return v;
    }

    private static Vector4 Mul4x4(double[,] A, Vector4 x)
    {
        return new Vector4(
            (float)(A[0,0]*x.x + A[0,1]*x.y + A[0,2]*x.z + A[0,3]*x.w),
            (float)(A[1,0]*x.x + A[1,1]*x.y + A[1,2]*x.z + A[1,3]*x.w),
            (float)(A[2,0]*x.x + A[2,1]*x.y + A[2,2]*x.z + A[2,3]*x.w),
            (float)(A[3,0]*x.x + A[3,1]*x.y + A[3,2]*x.z + A[3,3]*x.w)
        );
    }

    // q = (w,x,y,z) -> rotation matrix
    private static Matrix4x4 MatrixFromQuaternion(Vector4 q)
    {
        float w = q.x, x = q.y, y = q.z, z = q.w;
        float ww = w*w, xx = x*x, yy = y*y, zz = z*z;

        Matrix4x4 R = Matrix4x4.identity;
        R.m00 = ww + xx - yy - zz;
        R.m01 = 2f*(x*y - w*z);
        R.m02 = 2f*(x*z + w*y);

        R.m10 = 2f*(x*y + w*z);
        R.m11 = ww - xx + yy - zz;
        R.m12 = 2f*(y*z - w*x);

        R.m20 = 2f*(x*z - w*y);
        R.m21 = 2f*(y*z + w*x);
        R.m22 = ww - xx - yy + zz;

        R.m03 = 0f; R.m13 = 0f; R.m23 = 0f;
        R.m30 = 0f; R.m31 = 0f; R.m32 = 0f; R.m33 = 1f;
        return R;
    }

    private static float Determinant3x3(Matrix4x4 M)
    {
        float a = M.m00, b = M.m01, c = M.m02;
        float d = M.m10, e = M.m11, f = M.m12;
        float g = M.m20, h = M.m21, i = M.m22;
        return a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    }
}
