#ifndef DELAUNAY_H
#define DELAUNAY_H

#include "../../src/Geom2DTest/Geom2DTest.h"
#include "../PointCloud/PointCloud.h"

#include <memory>

class Mesh
{
};

struct DelaunayTriangle
{
	// vertices
	glm::vec3 v1, v2, v3;

	// parent and children
	DelaunayTriangle* parent;
	std::vector< DelaunayTriangle* > children;

	// Clear
	void Clear()
	{
		parent = nullptr;
		children.clear();
	}
};


class Delaunay
{
	// tracks algorithm iteration
	size_t iteration = -1;

	// root triangle
	DelaunayTriangle* rootTriangle = nullptr;

	// pool of triangles
	size_t MAX_TRIANGLES = 1024;
	std::vector<DelaunayTriangle> triangles;

public:

	Delaunay() 
	{
		// init pool
		triangles.resize(MAX_TRIANGLES);
	}
	~Delaunay() {}

	// Clear
	void Clear()
	{
		for (auto& triangle : triangles)
		{
			triangle.Clear();
		}

		triangles.clear();
		rootTriangle = nullptr;
		iteration = -1;
	}

	// Triangulate
	void Triangulate(const PointCloud& pointCloud, Mesh& outMesh)
	{
		// clear current triangulation
		Clear();

		// determine root triangle
		DetermineRootTriangle(pointCloud);

		// add points to triangulation
		AddPointsToTriangulation(pointCloud);

		// discard redundant triangles, those that do not belong to the triangulation
		DiscardRedundantTriangles();

		// Create mesh from triangulation
		CreateMeshFromTriangulation(outMesh);
	}

	// Triangulate by iterations (step by step)
	void TriangulateByIterations(const PointCloud& pointCloud, Mesh& outMesh)
	{
		if (iteration == -1)
		{
			// Determine the root triangle in the first iteration
			DetermineRootTriangle(pointCloud);
		}
		else if (iteration < pointCloud.Points().size())
		{
			// add another point to the triangulation
			AddPointToTriangulation(pointCloud.Points()[iteration]);
		}
		else
		{
			// discard redundant triangles and create the mesh
			DiscardRedundantTriangles();
			CreateMeshFromTriangulation(outMesh);
		}

		iteration++;
	}

	// getters
	DelaunayTriangle* RootTriangle() const { return rootTriangle; }

	// Expansion for the root triangle
	static const float s_rootTriangleExpansion;

private:

	// Determine root triangle
	void DetermineRootTriangle(const PointCloud& pointCloud)
	{
		// get the bounding box of the point cloud
		glm::vec3 topLeft;
		glm::vec3 bottomRight;
		pointCloud.GetBoundingBox(topLeft, bottomRight, s_rootTriangleExpansion);

		// calculate the super triangle that contains the bounding box
		// The intersection points of the 3 lines that define the triangle will be the vertices of the triangle

		// calculate the lines that forms the triangle
		float A1, B1, C1;
		Geom2DTest::GetLine(topLeft, glm::vec3(topLeft.x + 1.0f, 0.0f, topLeft.z - 1.0f), A1, B1, C1);

		float A2, B2, C2;
		Geom2DTest::GetLine(glm::vec3(bottomRight.x, 0.0f, topLeft.z), glm::vec3(bottomRight.x - 1.0f, 0.0f, topLeft.z - 1.0f), A2, B2, C2);

		float A3, B3, C3;
		Geom2DTest::GetLine(glm::vec3(topLeft.x, 0.0f, bottomRight.z), bottomRight, A3, B3, C3);

		// calculate triangle vertices
		glm::vec3 v1, v2, v3;
		Geom2DTest::LinesIntersects(A1, B1, C1, A2, B2, C2, v1);
		Geom2DTest::LinesIntersects(A1, B1, C1, A3, B3, C3, v2);
		Geom2DTest::LinesIntersects(A2, B2, C2, A3, B3, C3, v3);

		// set root triangle
		rootTriangle = &triangles[0];
		rootTriangle->v1 = v1;
		rootTriangle->v2 = v2;
		rootTriangle->v3 = v3;
	}

	// Add points to triangulation
	void AddPointsToTriangulation(const PointCloud& pointCloud)
	{
		auto& points = pointCloud.Points();
		for (auto& point : points)
		{
			AddPointToTriangulation(point);
		}
	}

	// Add point to triangulation
	void AddPointToTriangulation(const glm::vec3& point)
	{
		// get the triangle in which the point lies
		DelaunayTriangle* triangle = GetTriangleWhereToAddPoint(point, rootTriangle);
		assert(triangle);
		if (!triangle)
		{
			// no triangle found
			return;
		}
		// check if the point lies in one of the edges of the triangle found
		if (IsPointInTriangleEdge(*triangle, point))
		{
			// split adjacent triangles
			SplitAdjacentTriangles(*triangle, point);
		}
		else
		{
			// split triangle
			SplitTriangle(*triangle, point);
		}
	}

	// Get Triangle where to add point
	DelaunayTriangle* GetTriangleWhereToAddPoint(const glm::vec3& point, DelaunayTriangle* triangle)
	{	
		// Point in triangle test to check the triangle that contains the point to be added
		if (Geom2DTest::PointInTriangle(point, triangle->v1, triangle->v2, triangle->v3))
		{
			if (triangle->children.size() == 0)
			{
				// Leaf... so return this one
				return triangle;
			}
			else
			{
				// recursively check which of the children contains the point
				auto& children = triangle->children;
				for (size_t i = 0; i < children.size(); i++)
				{
					DelaunayTriangle* child = GetTriangleWhereToAddPoint(point, children[i]);
					if (child)
					{
						return child;
					}
				}
			}
		}

		return nullptr;
	}

	// Is Point In Triangle Edge
	bool IsPointInTriangleEdge(DelaunayTriangle& triangle, const glm::vec3& point)
	{
		// Point in segment test to check if the points lies in any of the edges of the triangle
		if (	Geom2DTest::PointInLineSegment(point, triangle.v1, triangle.v2)
			||	Geom2DTest::PointInLineSegment(point, triangle.v2, triangle.v3)
			||	Geom2DTest::PointInLineSegment(point, triangle.v3, triangle.v1))
		{
			return true;
		}

		return false;
	}

	// Split Adjacent Triangles
	void SplitAdjacentTriangles(DelaunayTriangle& triangle, const glm::vec3& point)
	{
		// TO-DO
		printf("TO-DO: Delaunay::SplitAdjacentTriangles\n");
	}

	// Split Triangle
	void SplitTriangle(DelaunayTriangle& triangle, const glm::vec3& point)
	{
		// TO-DO
		printf("TO-DO: Delaunay::SplitTriangle\n");
	}

	// Discard redundant triangles
	void DiscardRedundantTriangles()
	{
		// TO-DO
		printf("TO-DO: Delaunay::DiscardRedundantTriangles\n");
	}

	// Create mesh from triangulation
	void CreateMeshFromTriangulation(Mesh& outMesh)
	{
		// TO-DO
		printf("TO-DO: Delaunay::CreateMeshFromTriangulation\n");
	}
};

const float Delaunay::s_rootTriangleExpansion = 50.0f;

#endif
