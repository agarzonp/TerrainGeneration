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
}

#endif
