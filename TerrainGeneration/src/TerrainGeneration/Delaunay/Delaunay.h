#ifndef DELAUNAY_H
#define DELAUNAY_H

#include "../../src/Geom2DTest/Geom2DTest.h"
#include "../PointCloud/PointCloud.h"

class Mesh
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

private:

	// Determine root triangle
	void DetermineRootTriangle(const PointCloud& pointCloud)
	{
		// TO-DO
		printf("TO-DO: Delauny::DetermineRootTriangle\n");
	}

	// Add points to triangulation
	void AddPointsToTriangulation(const PointCloud& pointCloud)
	{
		// TO-DO
		printf("TO-DO: Delauny::AddPointsToTriangulation\n");
	}

	// Discard redundant triangles
	void DiscardRedundantTriangles()
	{
		// TO-DO
		printf("TO-DO: Delauny::DiscardRedundantTriangles\n");
	}

	// Create mesh from triangulation
	void CreateMeshFromTriangulation(Mesh& outMesh)
	{
		// TO-DO
		printf("TO-DO: Delauny::CreateMeshFromTriangulation\n");
	}


};

#endif
