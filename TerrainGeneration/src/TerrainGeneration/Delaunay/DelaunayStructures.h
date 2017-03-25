#ifndef DELAUNAY_STRUCTURES_H
#define DELAUNAY_STRUCTURES_H

#include "glm/glm.hpp"
#include <vector>

struct DelaunayTriangle;
struct DelaunayEdge;

struct DelaunayVertex
{
	// vertex
	glm::vec3 v; 

	// edge whose origin is v
	DelaunayEdge* edge = nullptr; 

	// Index used to when exporting the vertex
	int exportIndex = -1;

	void Clear()
	{
		edge = nullptr;
		exportIndex = -1;
	}
};

struct DelaunayEdge
{
	// the matching "twin" half-edge of the opposing face
	DelaunayEdge* twin = nullptr; 

	// the next half-edge
	DelaunayEdge* next = nullptr; 

	// the origin of this half-edge
	DelaunayVertex* v = nullptr; 	
	
	// the face connected to this half edge
	DelaunayTriangle* face = nullptr; 

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

	// vertices (This is mainly for debugging purpose)
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

#endif // !DELAUNAY_STRUCTURES_H
