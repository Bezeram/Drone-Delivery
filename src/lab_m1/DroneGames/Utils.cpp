#include "Utils.h"

float dg::Lerp(float x0, float x1, float t)
{
    return x0 * (1 - t) + x1 * t;
}

float dg::Floor(float x)
{
    return std::floor(x);
}

glm::vec2 dg::Floor(glm::vec2 xy)
{
    return { std::floor(xy.x), std::floor(xy.y) };
}

float dg::Fract(float x)
{
    return x - std::floor(x);
}

glm::vec2 dg::Fract(glm::vec2 x)
{
    return { x.x - std::floor(x.x), x.y - std::floor(x.y) };
}

float dg::Random(glm::vec2 pos)
{
    return dg::Fract(std::sin(glm::dot(pos, glm::vec2(12.9898, 78.233))) * 43758.5453123);
}

float dg::Mix(float x00, float x10, float x01, float x11, glm::vec2 uv)
{
    // Clamp the blend factors (optional for safety)
    float u = dg::Clamp(uv.x, 0.f, 1.f);
    float v = dg::Clamp(uv.y, 0.f, 1.f);

    // Perform bilinear interpolation
    float mix0 = dg::Lerp(x00, x10, u);
    float mix1 = dg::Lerp(x01, x11, u);
    return dg::Lerp(mix0, mix1, v);
}

// 2D Noise based on Morgan McGuire @morgan3d
// https://www.shadertoy.com/view/4dS3Wd
float dg::Noise(const glm::vec2& pos)
{
    glm::vec2 i = dg::Floor(pos);
    glm::vec2 f = dg::Fract(pos);

    // Four corners in 2D of a tile
    float a = dg::Random(i);
    float b = dg::Random(i + glm::vec2(1, 0));
    float c = dg::Random(i + glm::vec2(0, 1));
    float d = dg::Random(i + glm::vec2(1, 1));

    // Smooth Interpolation

    // Cubic Hermine Curve. Same as SmoothStep()
    glm::vec2 uv = f * f * (3.f - 2.f * f);

    // Mix 4 coorners percentages
    return dg::Mix(a, b, c, d, uv);
}

