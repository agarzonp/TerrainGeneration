#ifndef DELAUNAY_H
#define DELAUNAY_H

#include "../../src/Geom2DTest/Geom2DTest.h"
#include "../PointCloud/PointCloud.h"

#include <memory>

class Mesh
{
};

struct DelaunayTriangle;
struct DelaunayEdge;

struct DelaunayVertex
{
	glm::vec3 v; // vertex

	DelaunayEdge* edge = nullptr; // edge whose origin is v

	void Clear()
	{
		edge = nullptr;
	}
};

struct DelaunayEdge
{
	DelaunayEdge* twin = nullptr; // the matching "twin" half-edge of the opposing face
	DelaunayEdge* next = nullptr; // the next half-edge

	DelaunayVertex* v = nullptr; // the origin of this half-edge
	DelaunayTriangle* face = nullptr; // the face connected to this half edge

	void Clear()
	{
		twin = nullptr;
		next = nullptr;
		v = nullptr;
		face = nullptr;
	}
};

struct DelaunayTriangle
{
	// edge belonging to the triangle
	DelaunayEdge* edge; 

	// vertices (Do we need this?)
	glm::vec3 v1, v2, v3;

	// parent and children
	DelaunayTriangle* parent = nullptr;
	std::vector< DelaunayTriangle* > children;

	// Clear
	void Clear()
	{
		parent = nullptr;
		children.clear();
		edge = nullptr;
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
	std::vector<DelaunayTriangle> trianglesPool;

	// pool of edges
	size_t MAX_EDGES = MAX_TRIANGLES * 6;
	std::vector<DelaunayEdge> edgesPool;

	// pool of vertices
	size_t MAX_VERTICES = MAX_TRIANGLES * 3;
	std::vector<DelaunayVertex> verticesPool;

	// for tracking pool usage
	size_t numDelaunayTriangleUsed = 0;
	size_t numDelaunayEdgeUsed = 0;
	size_t numDelaunayVertexUsed = 0;

public:

	Delaunay() 
	{
		// initialize pools
		trianglesPool.resize(MAX_TRIANGLES);
		edgesPool.resize(MAX_EDGES);
		verticesPool.resize(MAX_VERTICES);
	}
	~Delaunay() {}

	// Clear
	void Clear()
	{
		for (auto& triangle : trianglesPool)
		{
			triangle.Clear();
		}

		for (auto& edge: edgesPool)
		{
			edge.Clear();
		}

		for (auto& vertices: verticesPool)
		{
			vertices.Clear();
		}

		numDelaunayTriangleUsed = 0;
		numDelaunayEdgeUsed = 0;
		numDelaunayTriangleUsed = 0;
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

	// Get a new DelaunayTriangle from the pool
	DelaunayTriangle* GetNewDelaunayTriangle()
	{
		if (numDelaunayTriangleUsed >= MAX_TRIANGLES)
		{
			assert(false);
			return nullptr;
		}
		
		return &trianglesPool[numDelaunayTriangleUsed++];
	}

	// Get a new DelaunayEdge from the pool
	DelaunayEdge* GetNewDelaunayEdge()
	{
		if (numDelaunayEdgeUsed >= MAX_EDGES)
		{
			assert(false);
			return nullptr;
		}

		return &edgesPool[numDelaunayEdgeUsed++];
	}

	// Get a new DelaunayVertex from the pool
	DelaunayVertex* GetNewDelaunayVertex()
	{
		if (numDelaunayVertexUsed >= MAX_VERTICES)
		{
			assert(false);
			return nullptr;
		}

		return &verticesPool[numDelaunayVertexUsed++];
	}

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

		// set root triangle with adjacency information
		rootTriangle = GetNewDelaunayTriangle();
		rootTriangle->v1 = v1;
		rootTriangle->v2 = v2;
		rootTriangle->v3 = v3;

		// set adjacency information
		//
		// get new half-edges
		DelaunayEdge* edgeA = GetNewDelaunayEdge();
		DelaunayEdge* edgeB = GetNewDelaunayEdge();
		DelaunayEdge* edgeC = GetNewDelaunayEdge();

		// root face edge
		rootTriangle->edge = edgeA;

		// root half-edge faces
		edgeA->face = edgeB->face = edgeC->face = rootTriangle;

		// root half-edge order
		SetEdgesOrderRelationship(edgeA, edgeB, edgeC);

		// root half-edge twins
		SetEdgesTwinRelationship(edgeA, nullptr);
		SetEdgesTwinRelationship(edgeB, nullptr);
		SetEdgesTwinRelationship(edgeC, nullptr);

		// get new vertices
		DelaunayVertex* vertexA = GetNewDelaunayVertex();
		vertexA->v = v1;
		vertexA->edge = edgeA;

		DelaunayVertex* vertexB = GetNewDelaunayVertex();
		vertexB->v = v2;
		vertexB->edge = edgeB;

		DelaunayVertex* vertexC = GetNewDelaunayVertex();
		vertexC->v = v3;
		vertexC->edge = edgeC;

		// hald-edge start vertex
		SetEdgesVertexRelationship(edgeA, vertexA);
		SetEdgesVertexRelationship(edgeB, vertexB);
		SetEdgesVertexRelationship(edgeC, vertexC);
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
			SplitAdjacentTriangles(triangle, point);
		}
		else
		{
			// split triangle
			SplitTriangle(triangle, point);
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
	void SplitAdjacentTriangles(DelaunayTriangle* triangle, const glm::vec3& point)
	{
		// TO-DO
		printf("TO-DO: Delaunay::SplitAdjacentTriangles\n");
	}

	// Split Triangle
	void SplitTriangle(DelaunayTriangle* triangle, const glm::vec3& point)
	{
		// get 3 new DelaunayTriangle
		DelaunayTriangle* childA = GetNewDelaunayTriangle();
		DelaunayTriangle* childB = GetNewDelaunayTriangle();
		DelaunayTriangle* childC = GetNewDelaunayTriangle();

		// set children
		childA->v1 = point;
		childA->v2 = triangle->v1;
		childA->v3 = triangle->v2;

		childB->v1 = point;
		childB->v2 = triangle->v2;
		childB->v3 = triangle->v3;

		childC->v1 = point;
		childC->v2 = triangle->v3;
		childC->v3 = triangle->v1;

		// update adjacency information
		UpdateAdjacencyInformation(triangle, childA, childB, childC, point);

		// set parent-child relationship
		SetParentChildRelationship(triangle, childA);
		SetParentChildRelationship(triangle, childB);
		SetParentChildRelationship(triangle, childC);
	}

	// Update Adjacency Information
	void UpdateAdjacencyInformation(DelaunayTriangle* parent, DelaunayTriangle* childA, DelaunayTriangle* childB, DelaunayTriangle* childC, const glm::vec3& point)
	{
		// child face edge
		childA->edge = parent->edge;
		childB->edge = parent->edge->next;
		childC->edge = parent->edge->next->next;

		// get new half-edges
		DelaunayEdge* childA_edgeA = childA->edge;
		DelaunayEdge* childA_edgeB = GetNewDelaunayEdge();
		DelaunayEdge* childA_edgeC = GetNewDelaunayEdge();

		DelaunayEdge* childB_edgeA = childB->edge;
		DelaunayEdge* childB_edgeB = GetNewDelaunayEdge();
		DelaunayEdge* childB_edgeC = GetNewDelaunayEdge();

		DelaunayEdge* childC_edgeA = GetNewDelaunayEdge();
		DelaunayEdge* childC_edgeB = GetNewDelaunayEdge();
		DelaunayEdge* childC_edgeC = GetNewDelaunayEdge();

		// half-edge faces
		childA_edgeA->face = childA_edgeB->face = childA_edgeC->face = childA;
		childB_edgeA->face = childB_edgeB->face = childB_edgeC->face = childB;
		childC_edgeA->face = childC_edgeB->face = childC_edgeC->face = childC;

		// half-edge order
		SetEdgesOrderRelationship(childA_edgeA, childA_edgeB, childA_edgeC);
		SetEdgesOrderRelationship(childB_edgeA, childB_edgeB, childB_edgeC);
		SetEdgesOrderRelationship(childC_edgeA, childC_edgeB, childC_edgeC);

		// half-edge twins
		SetEdgesTwinRelationship(childA_edgeA, childA_edgeA->twin);
		SetEdgesTwinRelationship(childB_edgeA, childB_edgeA->twin);
		SetEdgesTwinRelationship(childC_edgeA, childC_edgeA->twin);
		SetEdgesTwinRelationship(childA_edgeB, childB_edgeC);
		SetEdgesTwinRelationship(childA_edgeC, childC_edgeB);
		SetEdgesTwinRelationship(childB_edgeB, childC_edgeC);

		// get new vertices
		DelaunayVertex* vertex = GetNewDelaunayVertex();
		vertex->v = point;
		vertex->edge = childA_edgeC;

		// hald-edge start vertex
		SetEdgesVertexRelationship(childA_edgeA, childA->edge->v);
		SetEdgesVertexRelationship(childA_edgeB, childB->edge->v);
		SetEdgesVertexRelationship(childA_edgeC, vertex);
		SetEdgesVertexRelationship(childB_edgeA, childB->edge->v);
		SetEdgesVertexRelationship(childB_edgeB, childC->edge->v);
		SetEdgesVertexRelationship(childB_edgeC, vertex);
		SetEdgesVertexRelationship(childC_edgeA, childC->edge->v);
		SetEdgesVertexRelationship(childC_edgeB, childA->edge->v);
		SetEdgesVertexRelationship(childC_edgeC, vertex);
	}

	// Set parent-child relationship
	void SetParentChildRelationship(DelaunayTriangle* parent, DelaunayTriangle* child)
	{
		parent->children.push_back(child);
		child->parent = parent;
	}

	// Set edges order relationship
	void SetEdgesOrderRelationship(DelaunayEdge* edgeA, DelaunayEdge* edgeB, DelaunayEdge* edgeC)
	{
		edgeA->next = edgeB;
		edgeB->next = edgeC;
		edgeC->next = edgeA;
	}

	// Set edges vertex relationship
	void SetEdgesVertexRelationship(DelaunayEdge* edge, DelaunayVertex* vertex)
	{
		edge->v = vertex;
	}

	// Set edges twin relationship
	void SetEdgesTwinRelationship(DelaunayEdge* edgeA, DelaunayEdge* edgeB)
	{
		if (edgeA) edgeA->twin = edgeB;
		if (edgeB) edgeB->twin = edgeA;
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
