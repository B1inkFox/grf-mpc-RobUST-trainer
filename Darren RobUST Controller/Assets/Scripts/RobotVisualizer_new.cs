using UnityEngine;
using Unity.Mathematics;
using System;

public class RobotVisualizer : MonoBehaviour
{
    [Header("Activation")]
    public bool isActive = true;

    [Header("Body Geometry (Ellipsoid)")]
    public Vector3 ellipsoidRadiiMeters = new Vector3(0.20f, 0.15f, 0.30f);
    public Material bodyMaterial;

    [Header("Force Capsule Settings")]
    [Tooltip("Meters per Newton for capsule length.")]
    public float metersPerNewton = 0.0025f;

    [Tooltip("Minimum capsule length (meters) for visibility.")]
    public float minCapsuleLength = 0.02f;

    [Tooltip("Capsule radius (meters).")]
    public float capsuleRadius = 0.02f;

    [Tooltip("Hide capsule when |F| < eps.")]
    public float forceEps = 1e-6f;

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
    private double3 beltOffsetBody = double3.zero;

    // Thread-safe cached inputs
    private readonly object dataLock = new object();
    private bool hasNewData = false;

    private double3 cachedComWorld;
    private quaternion cachedTrunkWorld;

    private double3 cachedBeltForceWorld;
    private double3 cachedTotalForceWorld;

    private double3 cachedCop0World, cachedGrf0World;
    private double3 cachedCop1World, cachedGrf1World;

    // ============ Public API ============

    public bool Initialize(in double3 beltCenterOffsetBodyFrame)
    {
        beltOffsetBody = beltCenterOffsetBodyFrame;

        BuildSceneGraph();
        ApplyBodyEllipsoidScale();

        hasNewData = false;
        return true;
    }

    public void PushState(
        in double3 comWorld,
        in quaternion trunkWorld,
        in double3 beltForceWorld,
        in double3 totalForceWorld,
        in double3 cop0World, in double3 grf0World,
        in double3 cop1World, in double3 grf1World)
    {
        lock (dataLock)
        {
            cachedComWorld = comWorld;
            cachedTrunkWorld = trunkWorld;

            cachedBeltForceWorld = beltForceWorld;
            cachedTotalForceWorld = totalForceWorld;

            cachedCop0World = cop0World;
            cachedGrf0World = grf0World;

            cachedCop1World = cop1World;
            cachedGrf1World = grf1World;

            hasNewData = true;
        }
    }

    private void Update()
    {
        if (!isActive || bodyRoot == null) return;

        double3 comWorld;
        quaternion trunkWorld;
        double3 beltForceWorld, totalForceWorld;
        double3 cop0World, grf0World, cop1World, grf1World;

        lock (dataLock)
        {
            if (!hasNewData) return;

            comWorld = cachedComWorld;
            trunkWorld = cachedTrunkWorld;

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
        bodyRoot.rotation = ToUnityQuaternion(trunkWorld);

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

        if (bodyMaterial != null)
        {
            var r = ellipsoidGO.GetComponent<MeshRenderer>();
            if (r != null) r.material = bodyMaterial;
        }

        // Belt anchor (child of body)
        var beltAnchorGO = new GameObject("BeltAnchor");
        beltAnchorTf = beltAnchorGO.transform;
        beltAnchorTf.SetParent(bodyRoot, false);

        // Force capsules
        beltForceCapsuleTf = CreateForceCapsule("BeltForceCapsule", rootGO.transform).transform;
        totalForceCapsuleTf = CreateForceCapsule("TotalForceCapsule", rootGO.transform).transform;
        grf0CapsuleTf = CreateForceCapsule("GRF0_Capsule", rootGO.transform).transform;
        grf1CapsuleTf = CreateForceCapsule("GRF1_Capsule", rootGO.transform).transform;
    }

    private void ApplyBodyEllipsoidScale()
    {
        Vector3 radii = ellipsoidRadiiMeters;
        ellipsoidTf.localScale = new Vector3(2f * radii.x, 2f * radii.y, 2f * radii.z);
    }

    // ============ Capsule Force Rendering ============

    private GameObject CreateForceCapsule(string name, Transform parent)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        go.name = name;
        go.transform.SetParent(parent, false);
        DestroyColliderIfExists(go);

        // Default small so it doesn't flash huge before first update
        float L = Mathf.Max(minCapsuleLength, 2f * capsuleRadius);
        ApplyCapsuleScale(go.transform, L);

        return go;
    }

    /// <summary>
    /// Updates a capsule that represents a WORLD-frame force vector.
    /// The capsule originates at 'anchorWorld' and extends along the force direction.
    /// </summary>
    private void UpdateForceCapsuleWorld(Transform capsuleTf, Vector3 anchorWorld, in double3 forceWorld)
    {
        float mag = (float)math.length(forceWorld);

        if (mag < forceEps)
        {
            if (capsuleTf.gameObject.activeSelf) capsuleTf.gameObject.SetActive(false);
            return;
        }
        if (!capsuleTf.gameObject.activeSelf) capsuleTf.gameObject.SetActive(true);

        Vector3 dir = ToVector3(forceWorld / math.max(1e-12, (double)mag));

        // Length scales with magnitude
        float L = mag * metersPerNewton + 2f * capsuleRadius;

        // Orient: capsule's local Y axis points along dir
        capsuleTf.rotation = Quaternion.FromToRotation(Vector3.up, dir);

        // Place center so capsule starts at anchorWorld
        capsuleTf.position = anchorWorld + 0.5f * L * dir;

        // Scale to represent length and radius
        ApplyCapsuleScale(capsuleTf, L);
    }

    /// <summary>
    /// Applies capsule radius and length. Unity capsule is oriented along Y.
    /// Approximation: treat "unit capsule" height as 2, so scale.y = L/2.
    /// </summary>
    private void ApplyCapsuleScale(Transform capsuleTf, float lengthMeters)
    {
        float diameter = 2f * capsuleRadius;
        capsuleTf.localScale = new Vector3(diameter, 0.5f * lengthMeters, diameter);
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
