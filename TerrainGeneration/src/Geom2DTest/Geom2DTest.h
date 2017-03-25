#ifndef GEOM_2D_TEST_H
#define GEOM_2D_TEST_H

#include "glm/glm.hpp"

namespace Geom2DTest
{
	// Get Line (General equation line parameters)
	void GetLine(const glm::vec3& v1, const glm::vec3& v2, float& A, float& B, float& C)
	{
		glm::vec3 v = v2 - v1;

		A = v.z;
		B = -v.x;
		C = (v.x * v1.z) - (v.z * v1.x);
	}

	// Get perpendicular to line (General equation line parameters)
	void GetPerpendicularToLineThroughPoint(float A, float B, float C, const glm::vec3& p, float& PA, float& PB, float& PC)
	{
		PA = -B;
		PB = A;
		PC = -PA * p.x - PB * p.z;
	}

	// Square distance from point to line
	float SqrDistanceFromPointToLine(const glm::vec3& p, const glm::vec3& v1, const glm::vec3& v2)
	{
		float A, B, C;
		GetLine(v1, v2, A, B, C);

		return((A * p.x + B * p.z + C) * (A * p.x + B * p.z + C) / (A * A + B * B));
	}

	// Point in circle
	bool PointInCircle(const glm::vec3& p, const glm::vec3& c, float r)
	{
		float x = p.x - c.x;
		float z = p.z - c.z;

		return (x*x + z*z <= r * r);
	}

	// Point in triangle
	bool PointInTriangle(const glm::vec3& p, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3)
	{
		float A1, B1, C1;
		float A2, B2, C2;
		float A3, B3, C3;

		GetLine(v1, v2, A1, B1, C1);
		GetLine(v2, v3, A2, B2, C2);
		GetLine(v3, v1, A3, B3, C3);

		bool sign1, sign2, sign3;

		sign1 = ((A1 * p.x) + (B1 * p.z) + C1) > 0;
		sign2 = ((A2 * p.x) + (B2 * p.z) + C2) > 0;
		sign3 = ((A3 * p.x) + (B3 * p.z) + C3) > 0;

		return (sign1 == sign2 && sign2 == sign3);
	}

	// Point in line segment
	bool PointInLineSegment(const glm::vec3& p, const glm::vec3& v1, const glm::vec3& v2)
	{
		/*

		P = S + v * t

		t = (Px - Sx) / v1
		t = (Py - Sy) / v2

		point in segment if ---->  0 <= t <= 1
		*/

		float distanceToLine = SqrDistanceFromPointToLine(p, v1, v2);
		if (distanceToLine != 0)
		{
			return false;
		}

		glm::vec3 v = (v2 - v1);

		float t0, t1;

		if (v.x == 0)
		{
			t1 = (p.z - v1.z) / v.z;

			if (t1 >= 0 && t1 <= 1 && p.x == v1.x)
			{
				return true;
			}
		}
		else if (v.z == 0)
		{
			t0 = (p.x - v1.x) / v.x;

			if (t0 >= 0 && t0 <= 1 && p.z == v1.z)
			{
				return true;
			}
		}
		else
		{
			t0 = (p.x - v1.x) / v.x;
			t1 = (p.z - v1.z) / v.z;

			if (t0 == t1 && t0 >= 0 && t0 <= 1)
			{
				return true;
			}
		}

		return false;
	}

	// Lines intersect
	bool LinesIntersects(float A1, float B1, float C1, float A2, float B2, float C2, glm::vec3& intersection)
	{
		/***************************

		x = -B1 * y - C1 = -B2 * y - C2

		z = (A2 * C1 - A1 * C2 ) / (B2 * A1 - B1 * A2)

		****************************/

		float  denom = B2 * A1 - B1 * A2;
		if (denom == 0.0f)
		{
			return false;
		}

		intersection.z = (A2 * C1 - A1 * C2) / denom;
		intersection.x = (A2 == 0) ? (-B1 * intersection.z - C1) / A1 : (-B2 * intersection.z - C2) / A2;

		return true;
	}

	// Triangle circuncemter
	void TriangleCircumcenter(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, glm::vec3& c, float& r)
	{
		// general equation line for two segments
		float A1, B1, C1;
		GetLine(v1, v2, A1, B1, C1);

		float A2, B2, C2;
		GetLine(v2, v3, A2, B2, C2);

		// middle points of two segments
		glm::vec3 segmentV1V2MiddlePoint = (0.5f * (v1 + v2));
		glm::vec3 segmentV2V3MiddlePoint = (0.5f * (v2 + v3));

		// perpendicular lines of two segments
		float PA1, PB1, PC1;
		GetPerpendicularToLineThroughPoint(A1, B1, C1, segmentV1V2MiddlePoint, PA1, PB1, PC1);

		float PA2, PB2, PC2;
		GetPerpendicularToLineThroughPoint(A2, B2, C2, segmentV2V3MiddlePoint, PA2, PB2, PC2);

		// circumcenter
		LinesIntersects(PA1, PB1, PC1, PA2, PB2, PC2, c);
		r = glm::distance(glm::vec2(c.x, c.z), glm::vec2(v3.x, v3.z));
	}
}

#endif
