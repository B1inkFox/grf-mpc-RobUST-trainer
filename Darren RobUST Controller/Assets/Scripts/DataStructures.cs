using UnityEngine;
using Unity.Mathematics; 

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
    public double3 Force;
    public double3 CenterOfPressure;

    public ForcePlateData(double3 force, double3 centerOfPressure)
    {
        Force = force;
        CenterOfPressure = centerOfPressure;
    }
}


[System.Serializable]
public struct RobotState
{
    // COM state (from COM tracker, treated as COM up to constant bias)
    public double3 comPosition;    // [m]
    public double3 comVelocity;    // [m/s]
    public quaternion trunkOrientation;
    public double3 totalGRF;       // sum of all foot forces [N]
    public double3 globalCOP;      // effective CoP in robot frame [m]

    public RobotState(double3 cp, double3 cv, quaternion to, double3 grf, double3 cop)
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
    public double3 Inertia;
    public double3 InertiaCovariance;

}

[System.Serializable]
public struct Wrench
{
    public double3 Force;
    public double3 Torque;
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
