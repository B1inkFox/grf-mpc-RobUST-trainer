using UnityEngine;
using Unity.Mathematics;
using System;

public class RobotVisualizer : MonoBehaviour
{
    [Header("Activation")]
    public bool isActive = true;

    [Header("Body Geometry (Ellipsoid)")]
    public Vector3 ellipsoidRadii = new Vector3(0.1f, 0.1f, 0.1f);

    [Header("Force Capsule Settings")]
    [Tooltip("Meters per Newton for capsule length.")]
    public float metersPerNewton = 0.0025f;

    [Tooltip("Capsule radius (meters).")]
    public float capsuleRadius = 0.01f;

    // Scene graph
    private Transform bodyRoot;
    private Transform ellipsoidTf;
    private Transform beltAnchorTf;
    // Force capsules (Transforms)
    private Transform beltForceCapsuleTf;
    private Transform totalForceCapsuleTf;
    private Transform grf0CapsuleTf;
    private Transform grf1CapsuleTf;

    // Config
    private double3 beltOffsetBody;
    // Thread-safe cached inputs
    private readonly object dataLock = new object();
    private bool hasNewData = false;

    private double3 cachedComWorld;
    private quaternion cachedTrunkOrientationWorld;
    private double3 cachedBeltForceWorld;
    private double3 cachedTotalForceWorld;
    private double3 cachedCop0World, cachedGrf0World;
    private double3 cachedCop1World, cachedGrf1World;

    // ============ Public API ============

    public bool Initialize(in double3 beltOffsetBodyFrame, in double3 bodyInertia)
    {
        beltOffsetBody = beltOffsetBodyFrame;
        ellipsoidRadii = new Vector3((float)bodyInertia.x, (float)bodyInertia.y, (float)bodyInertia.z);

        BuildSceneGraph();
    
        hasNewData = false;
        return true;
    }

    public void PushState(
        in double3 comWorld,
        in quaternion trunkOrientationWorld,
        in double3 beltForceWorld,
        in double3 cop0World, in double3 grf0World,
        in double3 cop1World, in double3 grf1World)
    {
        lock (dataLock)
        {
            cachedComWorld = comWorld;
            cachedTrunkOrientationWorld = trunkOrientationWorld;

            cachedBeltForceWorld = beltForceWorld;
            
            cachedCop0World = cop0World;
            cachedGrf0World = grf0World;

            cachedCop1World = cop1World;
            cachedGrf1World = grf1World;

            cachedTotalForceWorld = beltForceWorld + grf0World + grf1World;

            hasNewData = true;
        }
    }

    private void Update()
    {
        if (!isActive || bodyRoot == null) return;

        double3 comWorld;
        quaternion trunkOrientationWorld;
        double3 beltForceWorld, totalForceWorld;
        double3 cop0World, grf0World, cop1World, grf1World;

        lock (dataLock)
        {
            if (!hasNewData) return;

            comWorld = cachedComWorld;
            trunkOrientationWorld = cachedTrunkOrientationWorld;

            beltForceWorld = cachedBeltForceWorld;
            totalForceWorld = cachedTotalForceWorld;

            cop0World = cachedCop0World;
            grf0World = cachedGrf0World;

            cop1World = cachedCop1World;
            grf1World = cachedGrf1World;

            hasNewData = false;
        }

        // 1) Body pose
        bodyRoot.position = ToVector3(comWorld);
        bodyRoot.rotation = ToUnityQuaternion(trunkOrientationWorld);

        // 2) Belt anchor (fixed offset in BODY frame)
        beltAnchorTf.localPosition = ToVector3(beltOffsetBody);

        // 3) Force capsules
        // Belt force: anchor is beltAnchorTf.position in WORLD
        UpdateForceCapsuleWorld(beltForceCapsuleTf, beltAnchorTf.position, beltForceWorld);

        // Total force: anchor is CoM in WORLD
        UpdateForceCapsuleWorld(totalForceCapsuleTf, bodyRoot.position, totalForceWorld);

        // GRFs: anchors are CoPs in WORLD
        UpdateForceCapsuleWorld(grf0CapsuleTf, ToVector3(cop0World), grf0World);
        UpdateForceCapsuleWorld(grf1CapsuleTf, ToVector3(cop1World), grf1World);
    }

    // ============ Scene Graph ============

    private void BuildSceneGraph()
    {
        var rootGO = new GameObject("RobUST_VisualizationRoot");
        rootGO.transform.SetParent(this.transform, false);

        // Body root
        var bodyGO = new GameObject("BodyRoot");
        bodyRoot = bodyGO.transform;
        bodyRoot.SetParent(rootGO.transform, false);

        // Ellipsoid
        var ellipsoidGO = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        ellipsoidGO.name = "BodyEllipsoid";
        ellipsoidTf = ellipsoidGO.transform;
        ellipsoidTf.SetParent(bodyRoot, false);
        DestroyColliderIfExists(ellipsoidGO);
        Renderer r = ellipsoidGO.GetComponent<Renderer>();
        r.material.color = Color.blue;
        ellipsoidTf.localScale = new Vector3(2f * ellipsoidRadii.x, 2f * ellipsoidRadii.y, 2f * ellipsoidRadii.z);

        // Belt anchor (child of body)
        var beltAnchorGO = new GameObject("BeltAnchor");
        beltAnchorTf = beltAnchorGO.transform;
        beltAnchorTf.SetParent(bodyRoot, false);

        // Force capsules
        beltForceCapsuleTf = CreateForceCapsule("BeltForce_Capsule", rootGO.transform).transform;
        totalForceCapsuleTf = CreateForceCapsule("TotalForce_Capsule", rootGO.transform).transform;
        grf0CapsuleTf = CreateForceCapsule("GRF0_Capsule", rootGO.transform).transform;
        grf1CapsuleTf = CreateForceCapsule("GRF1_Capsule", rootGO.transform).transform;
    }

    // ============ Capsule Force Rendering ============

    /// <summary>
    /// Initializes force capsule. Default length = 0 (force capsule reduces into sphere)
    /// </summary>
    private GameObject CreateForceCapsule(string name, Transform parent)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        go.name = name;
        go.transform.SetParent(parent, false);
        Renderer r = go.GetComponent<Renderer>();
        r.material.color = Color.green;
        DestroyColliderIfExists(go);
        ApplyCapsuleScale(go.transform, 0);
        return go;
    }

    /// <summary>
    /// Updates a capsule that represents a WORLD-frame force vector.
    /// The capsule originates at 'anchorWorld' and extends along the force direction.
    /// </summary>
    private void UpdateForceCapsuleWorld(Transform capsuleTf, Vector3 anchorWorld, in double3 forceWorld)
    {
        float mag = (float)math.length(forceWorld);
        Vector3 dir = new Vector3();

        if (mag < 0.1)
        {
            mag = 0;
            dir = Vector3.up;
        } else
        {
            dir = ToVector3(forceWorld / (double)mag);            
        }

        // Length scales with magnitude
        float L = mag * metersPerNewton;

        // Orient: capsule's local Y axis points along dir
        capsuleTf.rotation = Quaternion.FromToRotation(Vector3.up, dir);

        // Place center so capsule starts at anchorWorld
        capsuleTf.position = anchorWorld + 0.5f * L * dir;

        // Scale to represent length and radius
        ApplyCapsuleScale(capsuleTf, L);
    }

    /// <summary>
    /// Applies capsule radius and length. Unity capsule is oriented along Y.
    /// Capsule is of lengthMeters+2*radius in length.
    /// </summary>
    private void ApplyCapsuleScale(Transform capsuleTf, float lengthMeters)
    {
        capsuleTf.localScale = new Vector3(2f * capsuleRadius, 0.5f * lengthMeters + capsuleRadius, 2f * capsuleRadius);
    }

    private static void DestroyColliderIfExists(GameObject go)
    {
        var c = go.GetComponent<Collider>();
        if (c != null) Destroy(c);
    }

    private static Vector3 ToVector3(in double3 v) => new Vector3((float)v.x, (float)v.y, (float)v.z);

    private static Quaternion ToUnityQuaternion(in quaternion q)
        => new Quaternion(q.value.x, q.value.y, q.value.z, q.value.w);
}
