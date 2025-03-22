#version 330

in vec2 vertexPosition;
in vec3 vertexNormal;

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
    vec3 colorDark = vec3(0.31, 0.145, 0);
    vec3 colorLight = vec3(0.529, 0.243, 0);

    float noise = noise(vertexPosition * 5.0) * 0.5 + 0.5;

    vec3 lightDir1 = normalize(vec3(0.1, -0.5, 4));
    vec3 lightDir2 = normalize(vec3(0.1, -0.5, -3));
    float lightColor1 = max(dot(vertexNormal, lightDir1), 0.6);
    float lightColor2 = max(dot(vertexNormal, lightDir2), 0.6);
    vec3 color = mix(colorLight, colorDark, noise) * max(lightColor1, lightColor2);
    
    gl_FragColor = vec4(color, 1.0);
}
