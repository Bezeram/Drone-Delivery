#pragma once

#include "components/simple_scene.h"
#include "core/gpu/mesh.h"
#include "lab_m1/DroneGames/Utils.h"

#include <vector>
#include <iostream>
#include <limits>

namespace dg
{
	class Terrain
	{
	public:
		Terrain(size_t resolution, float length);
		Terrain(size_t resolution, float length, const glm::vec2& noisePosition, float noiseScalar);

		const std::vector<float>& GetHeightMap() const;
		const std::vector<glm::vec2>& GetTreesPositions() const;

		size_t GetResolution() const;
		float GetLength() const;
		float GetNoiseScalar() const;
		glm::vec2 GetNoiseOffset() const;

		float GetMaxHeight() const;
		float GetMinHeight() const;
		float GetTreesMinimumSeparation() const;

		float GetHeightScalar() const;

		float GetTreesDensity() const;

		glm::vec3 GetPosition(size_t x, size_t z) const;

		const std::vector<float>& UpdateNoiseOffset(const glm::vec2& noiseOffset);
		const std::vector<float>& UpdateNoiseScalar(float noiseScalar);
		const std::vector<float>& UpdateHeightScalar(float heightScalar);

		const std::vector<float>& GenerateTerrain();
		const std::vector<float>& GenerateTerrain(size_t resolution);
		const std::vector<float>& GenerateTerrain(float length);
		const std::vector<float>& GenerateTerrain(const glm::vec2& noisePosition, float scalar);
		const std::vector<float>& GenerateTerrain(size_t resolution, float length);
		const std::vector<float>& GenerateTerrain(size_t resolution, float length, const glm::vec2& noisePosition, float scalar);

		const std::vector<glm::vec2>& GenerateTrees();
		const std::vector<glm::vec2>& GenerateTrees(float density);

	private:
		float m_HeightScalar = 2.f;
		glm::vec2 m_NoiseOffset = { 0, 0 };
		float m_NoiseScalar = 0.24f;
		float m_Length;
		std::vector<float> m_HeightMap;

		float m_TreesMinimumSeparation = 2.f;
		float m_TreesDensity = 0.1f;
		std::vector<glm::vec2> m_TreesPositions;

		float m_MaxHeight = std::numeric_limits<float>::min();
		float m_MinHeight = std::numeric_limits<float>::max();
	};
}