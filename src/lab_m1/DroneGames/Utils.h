#pragma once

#include "components/simple_scene.h"
#include "core/gpu/mesh.h"

#include <vector>
#include <cmath>

namespace dg
{
    namespace Defaults
    {
        inline const int32_t TerrainResolution = 100;
        inline const int32_t TerrainLength = 80;
    }

    template<typename T>
    T Clamp(T x, T min, T max)
    {
        if (x < min)
            return min;
        if (x > max)
            return max;
        return x;
    }

    template<typename T>
    T ClampMin(T x, T min)
    {
        if (x < min)
            return min;
        return x;
    }

    template<typename T>
    T ClampMax(T x, T max)
    {
        if (x > max)
            return max;
        return x;
    }

    template<typename T>
	T Max(T a, T b)
	{
		return a > b ? a : b;
	}

    template<typename T>
    T Min(T a, T b)
    {
		return a < b ? a : b;
    }

    float Lerp(float x0, float x1, float t);
    float Mix(float x00, float x10, float x01, float x11, glm::vec2 uv);

    float Floor(float x);
    glm::vec2 Floor(glm::vec2 xy);
    float Fract(float x);
    glm::vec2 Fract(glm::vec2 x);

    float Random(glm::vec2 pos);
    float Noise(const glm::vec2& pos);
}

