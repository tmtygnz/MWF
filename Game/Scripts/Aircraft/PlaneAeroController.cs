using System;
using Godot;
using MiniWarfare.Game.Scripts.Utils;

public partial class PlaneAeroController : RigidBody3D
{
    [Export]
    public float MaxThrust;

    [Export]
    public float Throttle = 0f;

    [Export]
    public float wingSpanArea;

    [Export]
    public float turnPower;

    [Export]
    public Vector3 initialVelocity;

    [ExportCategory("Drag Configuration")]
    [Export]
    public float DragForward;

    [Export]
    public float DragBackwards;

    [Export]
    public float DragTop;

    [Export]
    public float DragDown;

    [Export]
    public float DragLeft;

    [Export]
    public float DragRight;

    [ExportCategory("AOA Lift Curve")]
    [Export]
    public Curve AOALiftCoeficient;
    [Export]
    public Curve AOALiftInducedDragCoeficient;

    private Vector3 lastVelocity = Vector3.Zero;
    private Vector3 localLinearVelocity = Vector3.Zero;
    private Vector3 localAngularVelocity = Vector3.Zero;
    private float currentGForce = 0f;
    private Vector3 acceleration = Vector3.Zero;

    private float angleOfAttack = 0f;
    private float angleOfAttackYaw = 0f;

    [ExportCategory("Orientation")]
    [Export]
    public Vector3 TurnVelocities;
    [Export]
    public Vector3 Macceleration;
    [Export]
    public bool IsDebug;

    private float airDensityCurrently = 1f;

    private float t;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        AngularDamp = 1f;
        LinearVelocity = initialVelocity;
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _PhysicsProcess(double delta)
    {
        _updatePhysicsStates((float)delta);
        _updateThrust((float)delta);
        _updateOrientation((float)delta);
        _updateDrag();
        _updateLift();

        int length = 5;

        DebugDraw3D.DrawLine(
            GlobalTransform.Origin,
            GlobalTransform.Origin + GlobalTransform.Basis * localLinearVelocity,
            Colors.DarkOrange
        );
        _drawUI();
        t += (float)delta;
    }

    private void _drawUI()
    {
        Camera3D camera = GetNode<Camera3D>("/root/Node3D/RigidBody3D2/PlaneCamera");
        Vector2 screenPos = camera.UnprojectPosition(GlobalTransform.Basis * LinearVelocity);

        Label tempLabel = GetNode<Label>("/root/Node3D/Control/CursorPosition");
        tempLabel.Position = screenPos;

        Label velocityLabel = GetNode<Label>("/root/Node3D/Control/Velocity");
        velocityLabel.Text = (localLinearVelocity.Length() / 1.94f).ToString() + " kts";
        _updateDrag();

        Label altitudeLabel = GetNode<Label>("/root/Node3D/Control/Altitude");
        altitudeLabel.Text = (GlobalPosition.Y).ToString();


        Label aoaLabel = GetNode<Label>("/root/Node3D/Control/AoA");
        aoaLabel.Text = $"{Mathf.RadToDeg(angleOfAttack).ToString():F2} {Mathf.RadToDeg(angleOfAttackYaw):F2}";
    }

    private float airDensity(float altitude)
    {
        const float rho = 1.225f;
        const float H = 8000;
        return rho * MathF.Exp(-altitude / H);
    }

    private float turnAuthority(float speed)
    {
        float k = 0.3f;
        float s = 200;
        float controlFactor = 1f / (1f + (float)Mathf.Exp(-k * (0.5) * (speed - s)));
        if (IsDebug)
            controlFactor = 1;
        return controlFactor;
    }

    private void _updatePhysicsStates(float dt)
    {
        Quaternion inverseRot = GlobalTransform.Basis.GetRotationQuaternion().Inverse();
        localLinearVelocity = inverseRot * LinearVelocity;
        localAngularVelocity = inverseRot * AngularVelocity;
        _calculateAOA();
        currentGForce = _calculateGForces(dt);
        lastVelocity = localLinearVelocity;
        airDensityCurrently = airDensity(GlobalPosition.Y);
    }

    private void _updateDrag()
    {
        Vector3 dragCoefficient = VectorUtils.ScaleEach(
            localLinearVelocity.Normalized(),
            DragForward, DragBackwards,
            DragTop, DragDown,
            DragLeft, DragRight);
        Vector3 drag = -localLinearVelocity.Normalized() *dragCoefficient.Length() * localLinearVelocity.LengthSquared();


        Quaternion rotation = GlobalTransform.Basis.GetRotationQuaternion();
        ApplyCentralForce(rotation * drag);

        DebugDraw3D.DrawArrow(GlobalTransform.Origin, GlobalTransform.Origin + drag, Colors.Green, 0.025f);
        GD.Print($"Drag: {drag} || [{t:F2}seconds]");
    }

    private void _updateThrust(float dt)
    {
        Throttle += Input.GetAxis("increase_thrust", "decrease_thrust") * dt;
        Throttle = Math.Clamp(Throttle, 0.0f, 1.0f);
        float thrustApplied = Throttle * MaxThrust;

        Quaternion rotation = this.GlobalTransform.Basis.GetRotationQuaternion();
        ApplyCentralForce(rotation * (thrustApplied * Vector3.Right));

        Vector3 thrustForce = rotation * (thrustApplied * Vector3.Right);
        DebugDraw3D.DrawArrow(GlobalTransform.Origin, GlobalTransform.Origin + thrustForce / Mass, Colors.Yellow, 0.2f);
        GD.Print($"Thrust: {thrustForce} || [{t:F2}seconds]");
    }

    private Vector3 CalculateLift(Vector3 rightAxis, float AoA, float power)
    {
        var liftVelocity = new Plane(rightAxis).Project(localLinearVelocity);
        var liftCoeficient = AOALiftCoeficient.Sample(Mathf.RadToDeg(AoA));
        float liftForce = 0.5f * liftCoeficient * airDensityCurrently * liftVelocity.LengthSquared() * wingSpanArea;

        var liftDirection = liftVelocity.Cross(rightAxis).Normalized();
        var lift = liftDirection * liftForce;

        var dragForce = liftCoeficient * liftCoeficient * 10f;
        var dragDirection = -liftVelocity.Normalized();
        //var inducedDrag = lift.LengthSquared() / (float.Pi * 0.7f * 3.2f * (0.5f * localLinearVelocity.LengthSquared() * airDensityCurrently)); // 0.7 oswalds number for testing set to that.
        //var inducedDragForce = dragDirection * inducedDrag;
        var inducedDrag = dragDirection * liftVelocity.LengthSquared() * dragForce;
        return lift;
    }
    private void _updateLift()
    {
        if (localLinearVelocity.LengthSquared() < 1f)
        {
            GD.Print("Shall not run lift");
            return ;
        }
        Vector3 upLift = CalculateLift(Vector3.Forward, angleOfAttack, wingSpanArea);
        Vector3 rudderLift = CalculateLift(Vector3.Up, angleOfAttackYaw, 50f);

        Quaternion rotation = GlobalTransform.Basis.GetRotationQuaternion();
        ApplyCentralForce(rotation * upLift);
        ApplyCentralForce(rotation * rudderLift);

        GD.Print($"LiftForce: {upLift} || [{t:F2}seconds]");
        GD.Print($"Stablizer: {rudderLift} || [{t:F2}seconds]");

        GD.Print($"AOA: {angleOfAttack} || AOAY: {angleOfAttackYaw} || [{t:F2}seconds]");


        DebugDraw3D.DrawArrow(GlobalTransform.Origin, GlobalTransform.Origin + (rotation * upLift) / Mass, Colors.Red, 0.025f);
        DebugDraw3D.DrawArrow(GlobalTransform.Origin, GlobalTransform.Origin + (rotation * rudderLift) / Mass, Colors.DeepPink, 0.025f);
    }

    private float _calculateSteerError(
        float targetAngularVelocity,
        float angularVelocity,
        float accelerationAngular,
        float dt
    )
    {
        float error = targetAngularVelocity - angularVelocity;
        float acc = accelerationAngular * dt;
        return Mathf.Clamp(error, -acc, acc);
    }

    private void _updateOrientation(float dt)
    {

        Vector3 userControls = new Vector3(
            Input.GetAxis("roll_left", "roll_right"),
            Input.GetAxis("yaw_left", "yaw_right"),
            Input.GetAxis("pitch_down", "pitch_up")
        );

        Vector3 targetav = userControls * TurnVelocities * turnPower;

        Vector3 av = new Vector3(
            Mathf.RadToDeg(localAngularVelocity.X),
            Mathf.RadToDeg(localAngularVelocity.Y),
            Mathf.RadToDeg(localAngularVelocity.Z)
        );

        Vector3 netTorqueWithCorrections = new Vector3(
            _calculateSteerError(
                targetav.X,
                localAngularVelocity.X,
                Macceleration.X,
                dt
            ),
            _calculateSteerError(
                targetav.Y,
                localAngularVelocity.Y,
                Macceleration.Y,
                dt
            ),
            _calculateSteerError(targetav.Z, localAngularVelocity.Z, Macceleration.Z, dt)
        );
        ApplyTorque(Transform.Basis * (netTorqueWithCorrections) * turnAuthority(localLinearVelocity.X) * turnPower);
    }

    private float _calculateGForces(double dt)
    {
        acceleration = (localLinearVelocity - lastVelocity) / (float)dt;
        float gForceExperienced = acceleration.Length() / 9.81f;
        return gForceExperienced;
    }

    private void _calculateAOA()
    {
        angleOfAttack = MathF.Atan2(-localLinearVelocity.Y, localLinearVelocity.X);
        angleOfAttackYaw = Mathf.Atan2(-localLinearVelocity.Z, localLinearVelocity.X);
    }
}
