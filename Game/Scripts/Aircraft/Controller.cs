using System;
using Godot;

public partial class Controller : RigidBody3D
{
    [Export]
    public float MaximumThrust = 100f;

    [Export]
    public float Throttle = 1;

    [Export]
    public Curve AOADependedCoeficient;

    [Export]
    public float liftPower;

    [Export]
    public float maxTurnRate;

    private float targetPitch = 0;
    private float targetRoll = 0;

    [ExportCategory("DragSettings")]
    [Export]
    public float DragXFB;

    [Export]
    public float DragYUD;

    [Export]
    public float DragZLR;

    public override void _Ready()
    {
        Input.MouseMode = Input.MouseModeEnum.Visible;
    }

    public override void _PhysicsProcess(double delta)
    {
        _updateThrust(delta);
        _calculateVelocity();
        _calculateAndApplyLift();
        _calculateAndApplyDrag();
        _doTurn();
    }

    private void _calculateVelocity()
    {
        float speed = LinearVelocity.Length() * 12149 / 6250;
        GD.Print($"[SPEED] Speed: {speed} knots || Position: {GlobalPosition}");
    }

    // Calcluates thrust input by depending on the throttle and maximum thrust.
    private void _updateThrust(double dt)
    {
        Throttle += Input.GetAxis("increase_thrust", "decrease_thrust") * 50f; // Adjust 50f for desired sensitivity
        Throttle = Mathf.Clamp(Throttle, 0f, 100f);
        Vector3 forceApplied = Transform.Basis.X * Throttle * MaximumThrust;
        ApplyForce(forceApplied);
        GD.Print($"[THRUST] Throttle: {Throttle} || Thrust Produced: {forceApplied} Newtons");
    }

    private void _calculateAndApplyLift()
    {
        float aoa = _calculateAOA();
        float liftValue =
            AOADependedCoeficient.SampleBaked(aoa) * LinearVelocity.LengthSquared() * liftPower;
        ApplyForce(Transform.Basis.Y * liftValue);
        GD.Print(
            $"[LIFT] Wing Induced Lift: {liftValue} || AOA: {aoa} deg || liftPower: {liftPower}"
        );
    }

    // TODO.
    private void _calculateAndApplyDrag()
    {
        float cdUsed = DragXFB;
        float aoa = _calculateAOA();
        if (Mathf.Abs(aoa) > 10)
            cdUsed = DragYUD;
        else
            cdUsed = DragXFB;

        float airDensity = 1.225f;
        float referenceArea = 20f;
        float speedSquared = LinearVelocity.LengthSquared();

        float dragMagnitude = 0.5f * airDensity * cdUsed * referenceArea * speedSquared;
        Vector3 dragForce = -LinearVelocity.Normalized() * dragMagnitude;

        ApplyForce(dragForce);
        GD.Print($"[DRAG] Wind induced drag: {dragForce} || Drag coeficient: {cdUsed}");
    }

    private void _calculateTurnError() { }

    private void _doTurn()
    {
        targetPitch += Input.GetAxis("up", "down");
        float targetPitchVelocity = targetPitch * maxTurnRate * 10f;
        ApplyTorque(targetPitchVelocity * Transform.Basis.Z);
        GD.Print(targetPitchVelocity);
    }

    // Calulates the angle of attack by subtracting the pitch to the angle of the velocity vector.
    private float _calculateAOA()
    {
        Vector3 velocityVector = LinearVelocity;
        float velocityVectorAngleDegrees = Mathf.RadToDeg(
            Mathf.Atan2(-velocityVector.Y, velocityVector.X)
        );

        Vector3 attackAngle = Transform.Basis.GetRotationQuaternion().GetEuler();
        Vector3 attackAngleDegrees = new Vector3(
            Mathf.RadToDeg(attackAngle.X),
            Mathf.RadToDeg(attackAngle.Y),
            Mathf.RadToDeg(attackAngle.Z)
        );

        return attackAngleDegrees.Z - velocityVectorAngleDegrees;
    }
}
