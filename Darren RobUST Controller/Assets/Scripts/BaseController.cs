using UnityEngine;

/// <summary>
/// Abstract base class for any high-level controller that outputs cable tensions.
/// Both MPCController and StabilityController derive from this.
/// </summary>
public abstract class BaseController
{
    public abstract void Initialize();
    public abstract ControllerOutput computeNextControl();
}
