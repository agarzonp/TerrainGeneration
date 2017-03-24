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
	size_t NUM_MAX_POINTS = 3;
	
	// the set of points
	std::vector<glm::vec3> points;

	// Bounding box (actually rectangle)
	glm::vec3 bboxTopLeft, bboxBottomRight;

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

			if (points.size() == 1)
			{
				bboxTopLeft = bboxBottomRight = points[0];
			}
			else
			{
				// update bounding box
				bboxTopLeft.x = std::min(bboxTopLeft.x, point.x);
				bboxBottomRight.x = std::max(bboxBottomRight.x, point.x);

				bboxTopLeft.z = std::min(bboxTopLeft.z, point.z);
				bboxBottomRight.z = std::max(bboxBottomRight.z, point.z);
			}
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

		PrintPoints();
	}

	// Get Bounding box
	void GetBoundingBox(glm::vec3& topLeft, glm::vec3& bottomRight, float expansion = 0.0f) const
	{
		topLeft.x = bboxTopLeft.x - expansion;
		topLeft.z = bboxTopLeft.z - expansion;

		bottomRight.x = bboxBottomRight.x + expansion;
		bottomRight.z = bboxBottomRight.z + expansion;
	}

	void CreateCustomTest()
	{
		// clear current set
		points.clear();

		AddPoint(glm::vec3(37.0f, 0.0f, 2.0f));
		AddPoint(glm::vec3(-11.0f, 0.0f, 10.0f));
		AddPoint(glm::vec3(19.0f, 0.0f, -10.0f));
	}

private:
	
	// Init 
	void Init()
	{
		points.reserve(NUM_MAX_POINTS);
	}

	void PrintPoints()
	{
		printf("-- PointCloud --\n");
		for (auto& point : points)
		{
			printf("P(%f, %f, %f)\n", point.x, point.y, point.z);
		}
		printf("-- PointCloud --\n");
	}
};

#endif // !POINT_CLOUD_H

