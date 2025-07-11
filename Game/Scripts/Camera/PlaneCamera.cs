using Godot;

public partial class PlaneCamera : Camera3D
{
    [Export] public Node3D target;
    [Export] public float distance = 10f;
    [Export] public float height = 2f;
    [Export] public float rotationSpeed = 0.01f;
    [Export] public float smoothingSpeed = 5f;

    private float yaw = 0f;
    private float pitch = 0f;
    private Vector2 lastMousePos;
    private bool rotating = false;

    public override void _Process(double delta)
    {
        if (target == null)
            return;

        HandleMouseInput();

        // Clamp pitch to avoid flipping
        pitch = Mathf.Clamp(pitch, -Mathf.Pi / 4, Mathf.Pi / 3);

        // Calculate offset based on yaw & pitch
        Vector3 offset = new Vector3(
            Mathf.Cos(yaw) * Mathf.Cos(pitch),
            Mathf.Sin(pitch),
            Mathf.Sin(yaw) * Mathf.Cos(pitch)
        ) * distance;

        Vector3 desiredPosition = target.GlobalTransform.Origin - offset + Vector3.Up * height;

        GlobalTransform = new Transform3D(
            GlobalTransform.Basis,
            GlobalTransform.Origin.Lerp(desiredPosition, (float)delta * smoothingSpeed)
        );

        LookAt(target.GlobalTransform.Origin + Vector3.Up * height, Vector3.Up);
    }

    private void HandleMouseInput()
    {
        if (Input.IsActionJustPressed("mouse_right"))
        {
            rotating = true;
            Input.MouseMode = Input.MouseModeEnum.Captured;
        }
        else if (Input.IsActionJustReleased("mouse_right"))
        {
            rotating = false;
            Input.MouseMode = Input.MouseModeEnum.Visible;
        }

        if (rotating)
        {
            Vector2 mouseDelta = Input.GetLastMouseVelocity();
            yaw -= mouseDelta.X * rotationSpeed;
            pitch -= mouseDelta.Y * rotationSpeed;
        }
    }
}
