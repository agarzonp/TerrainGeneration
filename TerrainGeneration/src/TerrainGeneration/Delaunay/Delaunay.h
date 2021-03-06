#ifndef DELAUNAY_H
#define DELAUNAY_H

#include "../../src/Geom2DTest/Geom2DTest.h"
#include "../PointCloud/PointCloud.h"
#include "DelaunayStructures.h"
#include "DelaunayTriangulationExporter.h"

#include <memory>

class Delaunay
{
	// tracks algorithm iteration
	size_t iteration = -1;

	// root triangle
	DelaunayTriangle* rootTriangle = nullptr;

	// pool of triangles
	size_t MAX_TRIANGLES = 1000000;
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

	// triangulation
	std::vector<DelaunayTriangle*> triangulation;

	// triangulation exporter
	DelaunayTriangulationExporter exporter;

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

		triangulation.clear();

		numDelaunayTriangleUsed = 0;
		numDelaunayEdgeUsed = 0;
		numDelaunayTriangleUsed = 0;
		rootTriangle = nullptr;
		iteration = -1;
	}

	// Triangulate
	void Triangulate(const PointCloud& pointCloud)
	{
		// clear current triangulation
		Clear();

		// determine root triangle
		DetermineRootTriangle(pointCloud);

		// add points to triangulation
		AddPointsToTriangulation(pointCloud);

		// get final triangulation
		GetFinalTriangulation(triangulation);
	}

	// Triangulate by iterations (step by step)
	void TriangulateByIterations(const PointCloud& pointCloud)
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
		else if (iteration == pointCloud.Points().size())
		{
			// get final triangulation
			GetFinalTriangulation(triangulation);
		}

		iteration++;

		//PrintDebugInfo();
	}

	// export triangulation
	void ExportTriangulation(std::string& filename)
	{
		exporter.Export(TriangulationExportFormat::WAVEFRONT_OBJ, triangulation, filename, true);
	}

	// get mesh from triangulation
	void GetMeshFromTriangulation(Mesh& mesh)
	{
		// export first to Wavefront format
		exporter.Export(TriangulationExportFormat::WAVEFRONT_OBJ, triangulation, std::string("tempWavefront"), false);

		// load the mesh
		mesh.LoadWavefrontObj(std::string("assets/Triangulations/tempWavefront.obj"));

		// remove temporal file
		std::remove("assets/Triangulations/tempWavefront.obj");

		// reset export indices
		for (auto& vertices : verticesPool)
		{
			vertices.exportIndex = -1;
		}
	}

	// getters
	DelaunayTriangle* RootTriangle() const { return rootTriangle; }
	const std::vector<DelaunayTriangle*>& Triangulation() const { return triangulation; }

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
		//assert(triangle);
		if (!triangle)
		{
			// no triangle found
			return;
		}
		// check if the point lies in one of the edges of the triangle found
		if (IsPointInTriangleEdge(triangle, point))
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
		const glm::vec3& v1 = triangle->edge->v->v;
		const glm::vec3& v2 = triangle->edge->next->v->v;
		const glm::vec3& v3 = triangle->edge->next->next->v->v;

		// Point in triangle test to check the triangle that contains the point to be added
		if (Geom2DTest::PointInTriangle(point, v1, v2, v3))
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
	bool IsPointInTriangleEdge(DelaunayTriangle* triangle, const glm::vec3& point)
	{
		const glm::vec3& v1 = triangle->edge->v->v;
		const glm::vec3& v2 = triangle->edge->next->v->v;
		const glm::vec3& v3 = triangle->edge->next->next->v->v;

		// Point in segment test to check if the points lies in any of the edges of the triangle
		if (	Geom2DTest::PointInLineSegment(point, v1, v2)
			||	Geom2DTest::PointInLineSegment(point, v2, v3)
			||	Geom2DTest::PointInLineSegment(point, v3, v1))
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

		// Legalize edges
		LegalizeEdges(point, childA);
		LegalizeEdges(point, childB);
		LegalizeEdges(point, childC);
	}

	// Update Adjacency Information
	void UpdateAdjacencyInformation(DelaunayTriangle* parent, DelaunayTriangle* childA, DelaunayTriangle* childB, DelaunayTriangle* childC, const glm::vec3& point)
	{
		// get new half-edges
		DelaunayEdge* childA_edgeA = GetNewDelaunayEdge();
		DelaunayEdge* childA_edgeB = GetNewDelaunayEdge();
		DelaunayEdge* childA_edgeC = GetNewDelaunayEdge();

		DelaunayEdge* childB_edgeA = GetNewDelaunayEdge();
		DelaunayEdge* childB_edgeB = GetNewDelaunayEdge();
		DelaunayEdge* childB_edgeC = GetNewDelaunayEdge();

		DelaunayEdge* childC_edgeA = GetNewDelaunayEdge(); 
		DelaunayEdge* childC_edgeB = GetNewDelaunayEdge();
		DelaunayEdge* childC_edgeC = GetNewDelaunayEdge();

		// child face edge
		childA->edge = childA_edgeA;
		childB->edge = childB_edgeA;
		childC->edge = childC_edgeA;

		// half-edge faces
		childA_edgeA->face = childA_edgeB->face = childA_edgeC->face = childA;
		childB_edgeA->face = childB_edgeB->face = childB_edgeC->face = childB;
		childC_edgeA->face = childC_edgeB->face = childC_edgeC->face = childC;

		// half-edge order
		SetEdgesOrderRelationship(childA_edgeA, childA_edgeB, childA_edgeC);
		SetEdgesOrderRelationship(childB_edgeA, childB_edgeB, childB_edgeC);
		SetEdgesOrderRelationship(childC_edgeA, childC_edgeB, childC_edgeC);

		// half-edge twins
		childA_edgeA->twin = parent->edge->twin;
		childB_edgeA->twin = parent->edge->next->twin;
		childC_edgeA->twin = parent->edge->next->next->twin;

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
		SetEdgesVertexRelationship(childA_edgeA, parent->edge->v);
		SetEdgesVertexRelationship(childA_edgeB, parent->edge->next->v);
		SetEdgesVertexRelationship(childA_edgeC, vertex);
		SetEdgesVertexRelationship(childB_edgeA, parent->edge->next->v);
		SetEdgesVertexRelationship(childB_edgeB, parent->edge->next->next->v);
		SetEdgesVertexRelationship(childB_edgeC, vertex);
		SetEdgesVertexRelationship(childC_edgeA, parent->edge->next->next->v);
		SetEdgesVertexRelationship(childC_edgeB, parent->edge->v);
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

	// Legalize edges
	void LegalizeEdges(const glm::vec3& point, DelaunayTriangle* triangle)
	{
		// find the edge to legalize
		DelaunayEdge* edge = nullptr; 
		if (triangle->edge->next->next->v->v == point)
		{
			edge = triangle->edge;
		}
		else if (triangle->edge->v->v == point)
		{
			edge = triangle->edge->next;
		}
		else if (triangle->edge->next->v->v == point)
		{
			edge = triangle->edge->next->next;
		}
		assert(edge);
		
		if (IsDelaunayEdgeIllegal(edge))
		{
			// flip edge
			FlipEdge(point, edge);
		}
	}

	// Flip edge
	void FlipEdge(const glm::vec3& point, DelaunayEdge* edge)
	{
		// Flip the edge by creating new triangles and edges and updating adjacency information

		// current state
		DelaunayEdge* edgeTwin = edge->twin;

		DelaunayTriangle* triangleA = edge->face;
		DelaunayTriangle* triangleB = edge->twin->face;

		DelaunayVertex* triangleA_vertexI = edge->v;
		DelaunayVertex* triangleA_vertexJ = edge->next->v;
		DelaunayVertex* triangleA_vertexK = edge->next->next->v;

		DelaunayVertex* triangleB_vertexI = edgeTwin->v;
		DelaunayVertex* triangleB_vertexJ = edgeTwin->next->v;
		DelaunayVertex* triangleB_vertexK = edgeTwin->next->next->v;

		// create two new triangles
		DelaunayTriangle* newTriangleA = GetNewDelaunayTriangle();
		newTriangleA->v1 = triangleA_vertexK->v;
		newTriangleA->v2 = triangleA_vertexI->v;
		newTriangleA->v2 = triangleA_vertexK->v;

		DelaunayTriangle* newTriangleB = GetNewDelaunayTriangle();
		newTriangleA->v1 = triangleB_vertexK->v;
		newTriangleA->v2 = triangleA_vertexI->v;
		newTriangleA->v2 = triangleB_vertexK->v;

		// set parent-child relationship
		SetParentChildRelationship(triangleA, newTriangleA);
		SetParentChildRelationship(triangleA, newTriangleB);
		SetParentChildRelationship(triangleB, newTriangleA);
		SetParentChildRelationship(triangleB, newTriangleB);

		// create new edges
		DelaunayEdge* triangleA_newEdgeA = GetNewDelaunayEdge();
		DelaunayEdge* triangleA_newEdgeB = GetNewDelaunayEdge();
		DelaunayEdge* triangleA_newEdgeC = GetNewDelaunayEdge();

		DelaunayEdge* triangleB_newEdgeA = GetNewDelaunayEdge();
		DelaunayEdge* triangleB_newEdgeB = GetNewDelaunayEdge();
		DelaunayEdge* triangleB_newEdgeC = GetNewDelaunayEdge();

		// set twin relationship
		SetEdgesTwinRelationship(triangleA_newEdgeA, edge->next->next->twin);
		SetEdgesTwinRelationship(triangleA_newEdgeB, edgeTwin->next->twin);
		SetEdgesTwinRelationship(triangleB_newEdgeA, edgeTwin->next->next->twin);
		SetEdgesTwinRelationship(triangleB_newEdgeB, edge->next->twin);
		SetEdgesTwinRelationship(triangleA_newEdgeC, triangleB_newEdgeC);

		// set order relationship
		SetEdgesOrderRelationship(triangleA_newEdgeA, triangleA_newEdgeB, triangleA_newEdgeC);
		SetEdgesOrderRelationship(triangleB_newEdgeA, triangleB_newEdgeB, triangleB_newEdgeC);

		// set faces
		newTriangleA->edge = triangleA_newEdgeA;
		newTriangleB->edge = triangleB_newEdgeA;

		triangleA_newEdgeA->face = triangleA_newEdgeB->face = triangleA_newEdgeC->face = newTriangleA;
		triangleB_newEdgeA->face = triangleB_newEdgeB->face = triangleB_newEdgeC->face = newTriangleB;

		// set vertex relationship
		SetEdgesVertexRelationship(triangleA_newEdgeA, triangleA_vertexK);
		SetEdgesVertexRelationship(triangleA_newEdgeB, triangleA_vertexI);
		SetEdgesVertexRelationship(triangleA_newEdgeC, triangleB_vertexK);
		SetEdgesVertexRelationship(triangleB_newEdgeA, triangleB_vertexK);
		SetEdgesVertexRelationship(triangleB_newEdgeB, triangleB_vertexI);
		SetEdgesVertexRelationship(triangleB_newEdgeC, triangleA_vertexK);

		// Legalize the edges of the new triangles
		LegalizeEdges(point, newTriangleA);
		LegalizeEdges(point, newTriangleB);
	}

	// Is DelaunayEdge Illegeal
	bool IsDelaunayEdgeIllegal(DelaunayEdge* edge)
	{
		DelaunayEdge* twin = edge->twin;
		if (!twin)
		{
			// the edge is not shared at all so no way to flip it
			return false;
		}

		// get triangle circumcenter
		float radius = 0.0f;
		glm::vec3 center;

		const glm::vec3& v1 = edge->v->v;
		const glm::vec3& v2 = edge->next->v->v;
		const glm::vec3& v3 = edge->next->next->v->v;
		Geom2DTest::TriangleCircumcenter(v1, v2, v3, center, radius);
		
		// check if the opposite vertex to v3 lies in the circle to determine that the edge is illegal
		if (Geom2DTest::PointInCircle(twin->next->next->v->v, center, radius))
		{
			return true;
		}

		return false;
	}

	// Get final triangulation
	void GetFinalTriangulation(std::vector<DelaunayTriangle*>& triangulation)
	{
		// discard those leaf triangles that do not share any of the vertexes of the root triangle
		const DelaunayVertex* v1 = rootTriangle->edge->v;
		const DelaunayVertex* v2 = rootTriangle->edge->next->v;
		const DelaunayVertex* v3 = rootTriangle->edge->next->next->v;

		for (size_t i = 0; i < numDelaunayTriangleUsed; i++)
		{
			DelaunayTriangle& triangle = trianglesPool[i];
			if (triangle.children.size() == 0)
			{
				if (	triangle.edge->v == v1 || triangle.edge->next->v == v1 || triangle.edge->next->next->v == v1 
					||	triangle.edge->v == v2 || triangle.edge->next->v == v2 || triangle.edge->next->next->v == v2
					||	triangle.edge->v == v3 || triangle.edge->next->v == v3 || triangle.edge->next->next->v == v3)
				{
					// discard triangle by breaking twin and parent-child relationship
					BreakTwinRelationship(triangle.edge);
					BreakTwinRelationship(triangle.edge->next);
					BreakTwinRelationship(triangle.edge->next->next);

					BreakParentChildRelationShip(&triangle);
				}
				else
				{
					// add triangle to triangulation
					triangulation.push_back(&triangle);
				}
			}
		}
	}

	// Break twin relationship
	void BreakTwinRelationship(DelaunayEdge* edge)
	{
		if (edge->twin) 
			edge->twin->twin = nullptr;
	}

	// Break parent relationship
	void BreakParentChildRelationShip(DelaunayTriangle* triangle)
	{
		auto& parentChildren = triangle->parent->children;
		auto childTriangle = std::find(parentChildren.begin(), parentChildren.end(), triangle);
		assert(childTriangle != parentChildren.end());
		parentChildren.erase(childTriangle);

		triangle->parent = nullptr;
	}

	// Print debug info
	void PrintDebugInfo()
	{
		printf("Iteration: %d\n", iteration);
 
		PrintTriangleInfo(rootTriangle);
	}

	void PrintTriangleInfo(const DelaunayTriangle* triangle)
	{
		auto& children = triangle->children;
		if (children.size() == 0)
		{
			PrintTriangleLeafInfo(triangle);
		}
		else
		{
			PrintTriangleNodeInfo(triangle);

			for (size_t i = 0; i < children.size(); i++)
			{
				PrintTriangleInfo(children[i]);
			}
		}
	}

	void PrintTriangleLeafInfo(const DelaunayTriangle* triangle)
	{
		glm::vec3 v1 = triangle->v1;
		glm::vec3 v2 = triangle->v2;
		glm::vec3 v3 = triangle->v3;

		printf("-- Leaf (%f, %f) (%f, %f) (%f, %f)\n", v1.x, v1.z, v2.x, v2.z, v3.x, v3.z);
		if (triangle->parent)
		{
			glm::vec3 pv1 = triangle->parent->v1;
			glm::vec3 pv2 = triangle->parent->v2;
			glm::vec3 pv3 = triangle->parent->v3;

			printf("  Parent (%f, %f) (%f, %f) (%f, %f)\n", pv1.x, pv1.z, pv2.x, pv2.z, pv3.x, pv3.z);
		}
		else
		{
			printf("  Parent NULL\n");
		}

		printf("   AdjacencyInfo\n");
		PrintTriangleEdgesInfo(triangle);
	}

	void PrintTriangleNodeInfo(const DelaunayTriangle* triangle)
	{
		glm::vec3 v1 = triangle->v1;
		glm::vec3 v2 = triangle->v2;
		glm::vec3 v3 = triangle->v3;

		printf("-- Node (%f, %f) (%f, %f) (%f, %f)\n", v1.x, v1.z, v2.x, v2.z, v3.x, v3.z);
		if (triangle->parent)
		{
			glm::vec3 pv1 = triangle->parent->v1;
			glm::vec3 pv2 = triangle->parent->v2;
			glm::vec3 pv3 = triangle->parent->v3;

			printf("  Parent (%f, %f) (%f, %f) (%f, %f)\n", pv1.x, pv1.z, pv2.x, pv2.z, pv3.x, pv3.z);
		}
		else
		{
			printf("  Parent NULL\n");
		}

		printf("   AdjacencyInfo--\n");
		PrintTriangleEdgesInfo(triangle);
	}

	void PrintTriangleEdgesInfo(const DelaunayTriangle* triangle)
	{
		glm::vec3 v1 = triangle->edge->v->v;
		glm::vec3 v2 = triangle->edge->next->v->v;
		glm::vec3 v3 = triangle->edge->next->next->v->v;

		printf("   EdgeA (%f, %f) (%f, %f)\n", v1.x, v1.z, v2.x, v2.z);
		printf("   EdgeB (%f, %f) (%f, %f)\n", v2.x, v2.z, v3.x, v3.z);
		printf("   EdgeC (%f, %f) (%f, %f)\n", v3.x, v3.z, v1.x, v1.z);

		if (triangle->edge->twin)
		{
			glm::vec3 tv1 = triangle->edge->twin->v->v;
			printf("   EdgeA (%f, %f) twin (%f, %f)\n", v1.x, v1.z, tv1.x, tv1.z);
		}
		else
		{
			printf("   EdgeA (%f, %f) twin NULL\n", v1.x, v1.z);
		}

		if (triangle->edge->next->twin)
		{
			glm::vec3 tv2 = triangle->edge->next->twin->v->v;
			printf("   EdgeB (%f, %f) twin (%f, %f)\n", v2.x, v2.z, tv2.x, tv2.z);
		}
		else
		{
			printf("   EdgeB (%f, %f) twin NULL\n", v2.x, v2.z);
		}

		if (triangle->edge->next->next->twin)
		{
			glm::vec3 tv3 = triangle->edge->next->next->twin->v->v;
			printf("   EdgeC (%f, %f) twin (%f, %f)\n", v3.x, v3.z, tv3.x, tv3.z);
		}
		else
		{
			printf("   EdgeC (%f, %f) twin NULL\n", v3.x, v3.z);
		}
	}

	

};

const float Delaunay::s_rootTriangleExpansion = 1000.0f;

#endif
