shader_type canvas_item;

uniform float pitch; // in degrees
uniform float roll;  // in degrees

void fragment() {
    vec2 uv = UV;

    // Scroll vertically for pitch (adjust to match texture layout)
    uv.y += pitch / 90.0; // tweak divisor as needed

    // Rotate for roll (manually apply rotation)
    float angle = radians(roll);
    vec2 centered = uv - vec2(0.5);
    float cosA = cos(angle);
    float sinA = sin(angle);

    vec2 rotated = vec2(
        centered.x * cosA - centered.y * sinA,
        centered.x * sinA + centered.y * cosA
    );

    uv = rotated + vec2(0.5);

    COLOR = texture(TEXTURE, uv);
}
