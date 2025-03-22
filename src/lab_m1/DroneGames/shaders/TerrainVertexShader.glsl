#version 330

// Input
layout(location = 0) in vec3 v_position;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_texture_coord;
layout(location = 3) in vec3 v_color;

// Uniform properties
uniform mat4 Model;
uniform mat4 View;
uniform mat4 Projection;
uniform vec2 NoiseOffset;
uniform float NoiseScalar;
uniform float HeightScalar;
uniform float MaxHeight;
uniform float MinHeight;

// 2D Random
float random(in vec2 st) 
{
    return fract(sin(dot(st.xy, vec2(12.9898,78.233))) * 43758.5453123);
}

// 2D Noise based on Morgan McGuire @morgan3d
// https://www.shadertoy.com/view/4dS3Wd
float noise(in vec2 st) 
{
    vec2 i = floor(st);
    vec2 f = fract(st);

    // Four corners in 2D of a tile
    float a = random(i);
    float b = random(i + vec2(1.0, 0.0));
    float c = random(i + vec2(0.0, 1.0));
    float d = random(i + vec2(1.0, 1.0));

    // Smooth Interpolation

    // Cubic Hermine Curve.  Same as SmoothStep()
    vec2 u = smoothstep(0.0, 1.0, f);

    // Mix 4 coorners percentages
    return mix(a, b, u.x) +
            (c - a) * u.y * (1.0 - u.x) +
            (d - b) * u.x * u.y;
}

// Send to fragment shader
out float height01;
out vec2 vertexPosition;

void main()
{
    vertexPosition = vec2(v_position.x, v_position.z);

    float noiseHeight = noise((v_position.xz + NoiseOffset) * NoiseScalar);
    float height = noiseHeight * HeightScalar;
    vec3 position = v_position + vec3(0, height, 0);

    height01 = (height - MinHeight) / (MaxHeight - MinHeight);

	gl_Position = Projection * View * Model * vec4(position, 1.0);
}
