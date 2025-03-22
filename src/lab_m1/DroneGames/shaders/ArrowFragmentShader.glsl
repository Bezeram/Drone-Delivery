#version 330

in vec3 vertexNormal;

uniform float rotation;

void main() {
    // Rotation matrix for the Y-axis
    mat3 rotationMatrix = mat3(
        cos(rotation), 0.0, sin(rotation),
        0.0,       1.0, 0.0,
        -sin(rotation), 0.0, cos(rotation)
    );

    vec3 rotatedNormal = rotationMatrix * vertexNormal;

    vec3 lightDir = vec3(0, -1.5, -3);
    vec3 objectColor = vec3(0, 0.3, 1);
    vec3 color = max(dot(rotatedNormal, lightDir), 0.7) * objectColor;
    gl_FragColor = vec4(color, 1.0);
}
