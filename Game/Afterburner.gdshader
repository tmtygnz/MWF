shader_type spatial;
render_mode blend_add, depth_draw_opaque, cull_disabled, unshaded;

uniform float magnitude : hint_range(0.0, 20.0) = 1.0;
uniform float nozzle_size : hint_range(0.1, 2.0) = 1.0;
uniform vec3 color : source_color = vec3(1.0, 0.5, 0.2);
uniform float noise_scale : hint_range(0.1, 100.0) = 2.0;
uniform float time_scale : hint_range(0.1, 100) = 1.0;
uniform float flame_length : hint_range(0.1, 1.0) = 0.5;

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(
        mix(hash(i + vec2(0.0, 0.0)), hash(i + vec2(1.0, 0.0)), u.x),
        mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), u.x),
        u.y
    );
}

float fbm(vec2 p, int octaves) {
    float v = 0.0;
    float a = 0.5;
    vec2 shift = vec2(100.0);
    for (int i = 0; i < octaves; ++i) {
        v += a * noise(p);
        p = p * 2.0 + shift;
        a *= 0.5;
    }
    return v;
}

void vertex() {
    // Scale the nozzle radius based on nozzle_size
    vec3 local_pos = VERTEX;
    local_pos.xy *= nozzle_size;
    VERTEX = local_pos;
}

void fragment() {
    // Use UV.y to represent distance along the flame (Z-axis)
    float t = UV.y;

    // Clamp flame effect to flame_length
    float flame_t = t / flame_length;
    float alpha_factor = 1.0 - step(1.0, flame_t); // 1.0 if flame_t <= 1.0, 0.0 otherwise

    vec2 uv = UV.xy * noise_scale;

    // Animate noise with time, reversed direction
    float time = TIME * time_scale;
    vec2 noise_uv = uv - vec2(0.0, time);

    // Generate noise for flame shape
    float n = fbm(noise_uv, 4);
    n = n * (1.0 - flame_t); // Fade noise towards the tip

    // Shape the flame: narrower at the tip, wider at the base
    float flame_shape = 1.0 - flame_t;
    flame_shape = pow(flame_shape, 2.0) * magnitude;

    // Combine noise and shape
    float alpha = flame_shape * n * alpha_factor;
    alpha = clamp(alpha, 0.0, 1.0);

    // Color gradient: brighter at the base, dimmer at the tip
    vec3 flame_color = color * (1.0 - flame_t * 0.5);

    // Set output
    ALBEDO = flame_color;
    ALPHA = alpha * magnitude;
}