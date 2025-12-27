using UnityEngine;

/// <summary>
/// A simple data structure to hold the position and rotation of a tracker.
/// NOTE: Data is stored in a RIGHT-HANDED coordinate system (OpenVR standard).
/// </summary>
[System.Serializable]
public struct TrackerData
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
/// </summary>
[System.Serializable]
public struct ForcePlateData
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
public struct Wrench
{
    public Vector3 Force;
    public Vector3 Torque;
}

/// <summary>
/// Abstract base class for any high-level controller that outputs cable tensions.
/// Both MPCController and StabilityController derive from this.
/// </summary>
public abstract class BaseController<T>
{
    public abstract void Initialize();
    public abstract T computeNextControl();
}
