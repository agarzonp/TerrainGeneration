#ifndef DELAUNAY_H
#define DELAUNAY_H

#include "../../src/Geom2DTest/Geom2DTest.h"
#include "../PointCloud/PointCloud.h"

class Mesh
{
};

class DelaunayTriangle
{
};

class Delaunay
{

public:

	Delaunay() {}
	~Delaunay() {}

	// Triangulate
	void Triangulate(const PointCloud& pointCloud, Mesh& outMesh)
	{
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
	void Triangulate(const PointCloud& pointCloud, Mesh& outMesh, bool firstIteration)
	{
		static size_t iteration = -1;

		if (firstIteration)
		{
			iteration = -1;
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

private:

	// Determine root triangle
	void DetermineRootTriangle(const PointCloud& pointCloud)
	{
		// TO-DO
		printf("TO-DO: Delaunay::DetermineRootTriangle\n");
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
		DelaunayTriangle* triangle = GetTriangleWhereToAddPoint(point);
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
	DelaunayTriangle* GetTriangleWhereToAddPoint(const glm::vec3& point)
	{
		DelaunayTriangle* triangle = nullptr;
		// TO-DO
		printf("TO-DO: Delaunay::GetTriangleWhereToAddPoint\n");
		return triangle;
	}

	// Is Point In Triangle Edge
	bool IsPointInTriangleEdge(DelaunayTriangle& triangle, const glm::vec3& point)
	{
		// TO-DO
		printf("TO-DO: Delaunay::IsPointInTriangleEdge\n");
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

#endif
