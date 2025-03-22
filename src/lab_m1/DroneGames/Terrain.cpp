#include "Terrain.h"

dg::Terrain::Terrain(size_t resolution, float length)
	: m_HeightMap(resolution * resolution)
	, m_Length(length)
{}

dg::Terrain::Terrain(size_t resolution, float length, const glm::vec2& noisePosition, float noiseScalar)
	: m_HeightMap(resolution)
	, m_Length(length)
	, m_NoiseOffset(noisePosition)
	, m_NoiseScalar(noiseScalar)
{}

const std::vector<float>& dg::Terrain::GetHeightMap() const
{
	return m_HeightMap;
}

const std::vector<glm::vec2>& dg::Terrain::GetTreesPositions() const
{
	return m_TreesPositions;
}

/**
* The resolution represents the number of points on a side of the terrain.
* The total amount of points is resolution sqaured.
* The terrain itself is a square.
* */
size_t dg::Terrain::GetResolution() const
{
	return std::sqrt(m_HeightMap.size());
}

float dg::Terrain::GetLength() const
{
	return m_Length;
}

float dg::Terrain::GetNoiseScalar() const
{
	return m_NoiseScalar;
}

glm::vec2 dg::Terrain::GetNoiseOffset() const
{
	return m_NoiseOffset;
}

float dg::Terrain::GetMaxHeight() const
{
	return m_MaxHeight;
}

float dg::Terrain::GetMinHeight() const
{
	return m_MinHeight;
}

float dg::Terrain::GetTreesMinimumSeparation() const
{
	return m_TreesMinimumSeparation;
}

float dg::Terrain::GetHeightScalar() const
{
	return m_HeightScalar;
}

float dg::Terrain::GetTreesDensity() const
{
	return m_TreesDensity;
}

glm::vec3 dg::Terrain::GetPosition(size_t x, size_t z) const
{
	x = dg::Clamp(x, (size_t)0, GetResolution() - 1);
	z = dg::Clamp(z, (size_t)0, GetResolution() - 1);

	float step = m_Length / float(GetResolution() - 1);
	return glm::vec3(x * step, m_HeightMap[x * GetResolution() + z], z * step);
}

const std::vector<float>& dg::Terrain::UpdateNoiseOffset(const glm::vec2& noiseOffset)
{
	m_NoiseOffset = noiseOffset;

	return GenerateTerrain();
}

const std::vector<float>& dg::Terrain::UpdateNoiseScalar(float noiseScalar)
{
	m_NoiseScalar = noiseScalar;

	return GenerateTerrain();
}

const std::vector<float>& dg::Terrain::UpdateHeightScalar(float heightScalar)
{
	m_HeightScalar = heightScalar;

	return GenerateTerrain();
}

const std::vector<float>& dg::Terrain::GenerateTerrain()
{
	// Generate points based on the noise function
	size_t resolution = GetResolution();
	float partialLength = m_Length / float(resolution - 1);

	m_MaxHeight = std::numeric_limits<float>::min();
	m_MinHeight = std::numeric_limits<float>::max();
	for (size_t y = 0; y < resolution; y++)
	{
		for (size_t x = 0; x < resolution; x++)
		{
			// Offset by the map position
			glm::vec2 positionVertex = glm::vec2(x, y) * partialLength;
			glm::vec2 pos = (m_NoiseOffset + positionVertex) * m_NoiseScalar;
			float height = dg::Noise(pos) * m_HeightScalar;

			m_HeightMap[x * resolution + y] = height;

			m_MaxHeight = std::max(m_MaxHeight, height);
			m_MinHeight = std::min(m_MinHeight, height);
		}
	}

	// print params
	//std::cout << "noise_offset: " << m_NoiseOffset << std::endl;
	//std::cout << "noise_scalar: " << m_NoiseScalar << std::endl;
	//std::cout << "height_scalar: " << m_HeightScalar << std::endl;

	return m_HeightMap;
}

const std::vector<glm::vec2>& dg::Terrain::GenerateTrees()
{
	m_TreesPositions.clear();

	size_t resolution = GetResolution();
	float partialLength = m_Length / float(resolution - 1);
	for (size_t z = 0; z < resolution; z++)
		for (size_t x = 0; x < resolution; x++)
			if (dg::Random(glm::vec2(x, z) * partialLength) < m_TreesDensity)
			{
				// check if tree is within some threshold to any other tree
				bool isClose = false;
				for (const auto& tree : m_TreesPositions)
				{
					float distance = glm::distance(glm::vec2(x, z), tree);
					if (distance < m_TreesMinimumSeparation)
					{
						isClose = true;
						break;
					}
				}

				if (!isClose)
					m_TreesPositions.push_back(glm::vec2(x, z));
			}

	return m_TreesPositions;
}

const std::vector<glm::vec2>& dg::Terrain::GenerateTrees(float density)
{
	m_TreesDensity = dg::Clamp(density, 0.f, 1.f);

	return GenerateTrees();
}

const std::vector<float>& dg::Terrain::GenerateTerrain(const glm::vec2& noisePosition, float scalar = 1)
{
	// Generate points based on the noise function
	m_NoiseOffset = noisePosition;
	m_NoiseScalar = scalar;

	return GenerateTerrain();
}

const std::vector<float>& dg::Terrain::GenerateTerrain(size_t resolution)
{
	resolution = dg::ClampMin(resolution, (size_t)2);
	m_HeightMap.resize(resolution * resolution);

	return GenerateTerrain();
}

const std::vector<float>& dg::Terrain::GenerateTerrain(float length)
{
	m_Length = dg::ClampMin(length, 0.01f);

	return GenerateTerrain();
}

const std::vector<float>& dg::Terrain::GenerateTerrain(size_t resolution, float length, const glm::vec2& noisePosition, float scalar = 1)
{
	resolution = dg::ClampMin(resolution, (size_t)2);
	m_HeightMap.resize(resolution * resolution);

	m_Length = dg::ClampMin(length, 0.01f);
	m_NoiseOffset = noisePosition;
	m_NoiseScalar = scalar;

	return GenerateTerrain();
}

const std::vector<float>& dg::Terrain::GenerateTerrain(size_t resolution, float length)
{
	resolution = dg::ClampMin(resolution, (size_t)2);
	m_HeightMap.resize(resolution);
	m_Length = dg::ClampMin(length, 0.01f);

	return GenerateTerrain();
}
