#version 330

in float height01;
in vec2 vertexPosition;

uniform vec2 targetPosition;
uniform float targetRadius;

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

void main() {
    vec3 color;
    float distToTarget = distance(vertexPosition, targetPosition);
    if (distToTarget > targetRadius)
    {
        vec3 colorMin = vec3(0, 0.741, 0.102);
        vec3 colorMax = vec3(0.529, 0.196, 0);

        vec3 colorGroundDark = vec3(0, 0.478, 0.067);
        vec3 colorGroundLight = vec3(0, 0.859, 0.122);

        float groundNoise = 
            noise(vec2(vertexPosition * 2)) * 0.6 + 
            noise(vec2(vertexPosition * 1)) * 0.3 + 
            noise(vec2(vertexPosition * 0.1)) * 0.1;
        groundNoise = (groundNoise + 1) / 2;

        vec3 heightColor = mix(colorMin, colorMax, height01);
        vec3 groundColor = mix(colorGroundDark, colorGroundLight, groundNoise);
        color = mix(groundColor, heightColor, 0.7);
    }
    else
    {
        vec3 colorCentre = vec3(0.8, 0.8, 0.8);
        vec3 colorEdge = vec3(1, 0, 0);
        
        // split into 5 bands of color
        float gradient = float(int(distToTarget / targetRadius * 5)) / 5.0;
        color = mix(colorCentre, colorEdge, gradient);
    }

    gl_FragColor = vec4(color, 1.0);
}
