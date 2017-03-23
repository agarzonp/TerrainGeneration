#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <algorithm>
#include <cassert>
#include <random>
#include <vector>

#include "glm/glm.hpp"

class PointCloud
{
	// maximum points allowed
	size_t NUM_MAX_POINTS = 10;
	
	// the set of points
	std::vector<glm::vec3> points;

public:
	PointCloud()
	{
		Init();
	}

	~PointCloud() {};

	const std::vector<glm::vec3>& Points() const { return points; }

	// Add a point 
	void AddPoint(const glm::vec3& point)
	{
		assert(points.size() < NUM_MAX_POINTS);
		if (points.size() < NUM_MAX_POINTS)
		{
			points.push_back(point);
		}
	}

	// Create Random Point Cloud between min and max
	void CreateRandom(const glm::vec3& min, const glm::vec3& max)
	{
		// clear current set
		points.clear();

		// create a uniform distribution for each coordinate and get a random coordinate
		std::random_device rd;
		std::mt19937 generator(rd());

		std::uniform_real_distribution<float> distributionX(min.x, max.x);
		std::uniform_real_distribution<float> distributionY(min.y, max.y);
		std::uniform_real_distribution<float> distributionZ(min.z, max.z);

		for (size_t i = 0; i < NUM_MAX_POINTS; i++)
		{
			float x = distributionX(generator);
			float y = distributionY(generator);
			float z = distributionZ(generator);

			// add point
			AddPoint(glm::vec3(x, y, z));
		}
	}

	// Get Bounding box
	void GetBoundingBox(glm::vec3& topLeft, glm::vec3& bottomRight) const
	{
		float minX = 0.0f;
		float maxX = 0.0f;
		float minZ = 0.0f;
		float maxZ = 0.0f;

		for (auto& point : points)
		{
			minX = std::min(minX, point.x);
			maxX = std::max(maxX, point.x);

			minZ = std::min(minZ, point.z);
			maxZ = std::max(maxZ, point.z);
		}
		
		topLeft.x = minX;
		topLeft.z = minZ;

		bottomRight.x = maxX;
		bottomRight.z = maxZ;
	}

private:
	
	// Init 
	void Init()
	{
		points.reserve(NUM_MAX_POINTS);
	}
};

#endif // !POINT_CLOUD_H

