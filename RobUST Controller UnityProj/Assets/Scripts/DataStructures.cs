using UnityEngine;
using Unity.Mathematics;
using Unity.Profiling;

public static class RobotProfiler
{
    public static readonly ProfilerCategory Workloads = new("Robot Thread Workloads");
    public static readonly ProfilerCategory Intervals = new("Robot Thread Intervals");
}
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
    public double3 CoP;

    public ForcePlateData(double3 force, double3 cop)
    {
        Force = force;
        CoP = cop;
    }
}

/// <summary>
/// Rigid body state for MPC prediction.
/// </summary>
[System.Serializable]
public struct RBState
{
    public double3 p;   // position [m]
    public double3 th;  // ZYX Euler angles [rad]
    public double3 v;   // linear velocity [m/s]
    public double3 w;   // angular velocity [rad/s]

    public RBState(double3 position, double3 eulerAngles, double3 linearVelocity, double3 angularVelocity)
    {
        p = position;
        th = eulerAngles;
        v = linearVelocity;
        w = angularVelocity;
    }
}


[System.Serializable]
public struct Wrench
{
    public double3 Force;
    public double3 Torque;

    public Wrench(double3 force, double3 torque)
    {
        Force = force;
        Torque = torque;
    }
}

/// <summary>
/// Abstract base class for any high-level controller that outputs cable tensions.
/// Both MPCController and StabilityController derive from this.
/// </summary>
public abstract class BaseController<T>
{
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
    
    /// <summary>Cable attachment points on belt, in end-effector frame [m]</summary>
    public readonly double3[] LocalAttachmentPoints;
    
    /// <summary>Belt center in end-effector frame [m]</summary>
    public readonly double3 BeltCenter_EE_Frame;
    
    
    // ============ User Parameters ============
    /// <summary>User body mass [kg]</summary>
    public readonly double UserMass;
    
    /// <summary>User shoulder width [m]</summary>
    public readonly double UserShoulderWidth;
    
    /// <summary>User trunk height from hip to shoulder [m]</summary>
    public readonly double UserHeight;
    
    /// <summary>Chest anterior-posterior distance [m]</summary>
    public readonly double ChestAPDistance;
    
    /// <summary>Chest medial-lateral distance [m]</summary>
    public readonly double ChestMLDistance;

    // ============ Force Plate Geometry ============
    public readonly double3 FP_BackLeft;
    public readonly double3 FP_BackRight;
    public readonly double3 FP_FrontLeft;
    public readonly double3 FP_FrontRight;

    // Full 8-cable hardware definition (Static Database from vive tracker measurement)
    private static readonly double3[] AllPulleyPositions = new double3[]
    {
        new double3(-0.8000, 1.650, 0.9875),   // 0: Front-Right Top (Motor 10)
        new double3(-0.7900, 0.0022, 0.9540),   // 1: Front-Left Top (Motor 5)
        new double3(0.9580, 0.0335, 0.9828),    // 2: Back-Left Top (Motor 4)
        new double3(0.9650, 1.6420, 1.0000),    // 3: Back-Right Top (Motor 11)
        new double3(-0.7850, 1.6750, -0.2920),  // 4: Front-Right Bottom (Motor 8)
        new double3(-0.7800, 0.0330, -0.3340),  // 5: Front-Left Bottom (Motor 7)
        new double3(0.9850, 0.0520, -0.5165),   // 6: Back-Left Bottom (Motor 2)
        new double3(0.9800, 1.6890, -0.4994)    // 7: Back-Right Bottom (Motor 13)
    };

    // Mapping from solver index to motor driver index for the full set
    // Corresponds to the order in AllPulleyPositions
    public static readonly int[] FullMotorMapping = new int[] { 9, 4, 3, 10, 7, 6, 1, 12 };

    public readonly int[] SolverToMotorMap;

    private RobUSTDescription(int numCables, double chestAP, double chestML, 
                              double userMass, double shoulderWidth, double userHeight)
    {
        NumCables = numCables;
        ChestAPDistance = chestAP;
        ChestMLDistance = chestML;
        UserMass = userMass;
        UserShoulderWidth = shoulderWidth;
        UserHeight = userHeight;

        // Force Plate Configuration ----
        double3 back_left_correction   = new double3( 0.03, -0.03, -0.01);
        double3 back_right_correction  = new double3( 0.03,  0.03, -0.01);
        double3 front_left_correction  = new double3(-0.03, -0.03, -0.01);
        double3 front_right_correction = new double3(-0.03,  0.03, -0.01);
        // vive tracker jig readings:
        FP_BackLeft   = new double3( 0.5385, 0.5793, -0.9520) + back_left_correction;
        FP_BackRight  = new double3( 0.5234, 1.1350, -0.9463) + back_right_correction;
        FP_FrontLeft  = new double3(-0.3202, 0.5510, -0.9525) + front_left_correction;
        FP_FrontRight = new double3(-0.3341, 1.1077, -0.9458) + front_right_correction;        
        
        FramePulleyPositions = new double3[numCables];
        LocalAttachmentPoints = new double3[numCables];
        SolverToMotorMap = new int[numCables];

        // Determine active subset based on requested numCables
        int[] activeIndices = numCables switch
        {
            8 => new int[] { 0, 1, 2, 3, 4, 5, 6, 7 },
            4 => new int[] { 0, 1, 2, 3 }, // Top cables only
            _ => throw new System.ArgumentException($"Unsupported cable count: {numCables}. valid options: 4, 8")
        };

        double halfML = chestML / 2.0;
        
        // Populate arrays based on active indices
        for (int i = 0; i < numCables; i++)
        {
            int srcIdx = activeIndices[i];
            
            FramePulleyPositions[i] = AllPulleyPositions[srcIdx];
            SolverToMotorMap[i] = FullMotorMapping[srcIdx];
            
            if (srcIdx == 0 || srcIdx == 4) LocalAttachmentPoints[i] = new double3(-halfML, -chestAP, 0);
            else if (srcIdx == 1 || srcIdx == 5) LocalAttachmentPoints[i] = new double3(halfML, -chestAP, 0);
            else if (srcIdx == 2 || srcIdx == 6) LocalAttachmentPoints[i] = new double3(halfML, 0, 0);
            else if (srcIdx == 3 || srcIdx == 7) LocalAttachmentPoints[i] = new double3(-halfML, 0, 0);
        }

        BeltCenter_EE_Frame = new double3(0, -chestAP / 2.0, 0);
    }

    /// <summary>
    /// Factory method to create RobUST description from belt/chest configuration.
    /// All allocations happen here at init - nothing at runtime.
    /// </summary>
    /// <param name="numCables">Number of cables in the system</param>
    /// <param name="chestAPDistance">Chest anterior-posterior distance [m]</param>
    /// <param name="chestMLDistance">Chest medial-lateral distance [m]</param>
    /// <param name="userMass">User body mass [kg]</param>
    /// <param name="shoulderWidth">User shoulder width [m]</param>
    /// <param name="userHeight">User trunk height hip-to-shoulder [m]</param>
    public static RobUSTDescription Create(int numCables, double chestAPDistance, double chestMLDistance,
                                           double userMass, double shoulderWidth, double userHeight)
    {
        return new RobUSTDescription(numCables, chestAPDistance, chestMLDistance, userMass, shoulderWidth, userHeight);
    }
}
