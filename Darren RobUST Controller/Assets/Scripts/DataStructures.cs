using UnityEngine;

/// <summary>
/// A simple data structure to hold the position and rotation of a tracker.
/// NOTE: Data is stored in a RIGHT-HANDED coordinate system (OpenVR standard).
/// This is a non-MonoBehaviour class, used for organizing data efficiently.
/// </summary>
[System.Serializable]
public class TrackerData
{
    /// <summary>
    /// The full 4x4 homogeneous transformation matrix representing the tracker's pose.
    /// NOTE: This matrix is in the RIGHT-HANDED coordinate system (OpenVR standard).
    /// </summary>
    public Matrix4x4 PoseMatrix;

    public TrackerData()
    {
        PoseMatrix = Matrix4x4.identity;
    }
}

/// <summary>
/// A simple data structure to hold force and moment data from a force plate.
/// This is a non-MonoBehaviour class, used for organizing data.
/// </summary>
[System.Serializable]
public class ForcePlateData
{
    public Vector3 Force;
    public Vector3 CenterOfPressure;

    public ForcePlateData(Vector3 force, Vector3 centerOfPressure)
    {
        Force = force;
        CenterOfPressure = centerOfPressure;
    }
}


[System.Serializable]
public struct RobotState
{
    // COM state (from COM tracker, treated as COM up to constant bias)
    public Vector3 comPosition;    // [m]
    public Vector3 comVelocity;    // [m/s]
    public Quaternion trunkOrientation;
    public Vector3 totalGRF;       // sum of all foot forces [N]
    public Vector3 globalCOP;      // effective CoP in robot frame [m]

    public RobotState(Vector3 cp, Vector3 cv, Quaternion to, Vector3 grf, Vector3 cop)
    {
        comPosition = cp;
        comVelocity = cv;
        trunkOrientation = to;
        totalGRF = grf;
        globalCOP = cop;
    }
}

[System.Serializable]
public struct Hyperparameter
{
    public double mass;
    public Vector3 Inertia;
    public Vector3 InertiaCovariance;

}

[System.Serializable]
public struct ControllerOutput
{
    public Vector3 comForce;
    public Vector3 comTorque;
}