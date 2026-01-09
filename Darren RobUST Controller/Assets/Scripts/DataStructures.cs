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

    public TrackerData(Matrix4x4 poseMatrix)
    {
        PoseMatrix = poseMatrix;
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

/// <summary>
/// Complete robot description for the RobUST cable-driven parallel robot.
/// Contains all constant parameters needed for kinematics, dynamics, and control.
/// Allocated once at startup - zero runtime allocations.
/// </summary>
public sealed class RobUSTDescription
{
    public readonly int NumCables;
    
    /// <summary>Pulley positions in robot frame [m] (double3 for SIMD)</summary>
    public readonly double3[] FramePulleyPositions;
    
    /// <summary>Pulley positions in robot frame [m] (Vector3 for Unity visualization)</summary>
    public readonly Vector3[] FramePulleyPositionsVec3;
    
    /// <summary>Cable attachment points on belt, in end-effector frame [m]</summary>
    public readonly double3[] LocalAttachmentPoints;
    
    /// <summary>Belt center in end-effector frame [m]</summary>
    public readonly double3 BeltCenter_EE_Frame;
    
    /// <summary>Chest anterior-posterior distance [m]</summary>
    public readonly double ChestAPDistance;
    
    /// <summary>Chest medial-lateral distance [m]</summary>
    public readonly double ChestMLDistance;

    private RobUSTDescription(int numCables, double chestAP, double chestML)
    {
        NumCables = numCables;
        ChestAPDistance = chestAP;
        ChestMLDistance = chestML;
        
        FramePulleyPositions = new double3[numCables];
        FramePulleyPositionsVec3 = new Vector3[numCables];
        LocalAttachmentPoints = new double3[numCables];

        // Fixed pulley positions relative to robot frame tracker (measured/calibrated)
        FramePulleyPositions[0] = new double3(-0.8114, 1.6556, 0.9400);   // Front-Right Top (Motor 10)
        FramePulleyPositions[1] = new double3(-0.8066, 0.0084, 0.8895);   // Front-Left Top (Motor 5)
        FramePulleyPositions[2] = new double3(0.9827, 0.0592, 0.9126);    // Back-Left Top (Motor 4)
        FramePulleyPositions[3] = new double3(0.9718, 1.6551, 0.9411);    // Back-Right Top (Motor 11)
        FramePulleyPositions[4] = new double3(-0.8084, 1.6496, -0.3060);  // Front-Right Bottom (Motor 8)
        FramePulleyPositions[5] = new double3(-0.7667, 0.0144, -0.3243);  // Front-Left Bottom (Motor 7)
        FramePulleyPositions[6] = new double3(0.9748, 0.0681, -0.5438);   // Back-Left Bottom (Motor 2)
        FramePulleyPositions[7] = new double3(0.9498, 1.6744, -0.5409);   // Back-Right Bottom (Motor 13)

        // Copy to Vector3 array for visualizer compatibility
        for (int i = 0; i < numCables; i++)
        {
            FramePulleyPositionsVec3[i] = new Vector3(
                (float)FramePulleyPositions[i].x,
                (float)FramePulleyPositions[i].y,
                (float)FramePulleyPositions[i].z);
        }

        // Local attachment points based on belt geometry relative to end-effector tracker
        double halfML = chestML / 2.0;
        LocalAttachmentPoints[0] = new double3(-halfML, -chestAP, 0);  // Front-Right
        LocalAttachmentPoints[1] = new double3(halfML, -chestAP, 0);   // Front-Left
        LocalAttachmentPoints[2] = new double3(halfML, 0, 0);          // Back-Left
        LocalAttachmentPoints[3] = new double3(-halfML, 0, 0);         // Back-Right
        // Repeat for bottom cables
        LocalAttachmentPoints[4] = new double3(-halfML, -chestAP, 0);  // Front-Right
        LocalAttachmentPoints[5] = new double3(halfML, -chestAP, 0);   // Front-Left
        LocalAttachmentPoints[6] = new double3(halfML, 0, 0);          // Back-Left
        LocalAttachmentPoints[7] = new double3(-halfML, 0, 0);         // Back-Right

        // Belt center in end-effector frame
        BeltCenter_EE_Frame = new double3(0, -chestAP / 2.0, 0);
    }

    /// <summary>
    /// Factory method to create RobUST description from belt/chest configuration.
    /// All allocations happen here at init - nothing at runtime.
    /// </summary>
    public static RobUSTDescription Create(int numCables, double chestAPDistance, double chestMLDistance)
    {
        return new RobUSTDescription(numCables, chestAPDistance, chestMLDistance);
    }
}
