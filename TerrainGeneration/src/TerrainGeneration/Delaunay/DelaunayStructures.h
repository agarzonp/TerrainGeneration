#ifndef DELAUNAY_STRUCTURES_H
#define DELAUNAY_STRUCTURES_H

#include "glm/glm.hpp"
#include <vector>

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
