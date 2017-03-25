#ifndef SPLINE_CAM_H
#define SPLINE_CAM_H

#include "../Input/Input.h"
#include "../Geom2DTest/Geom2DTest.h"
#include "../Mesh/Mesh.h"
#include "../Shaders/Shader.h"

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
#include "Camera/FreeCamera.h"

#include "PointCloud/PointCloud.h"

#include "Delaunay/Delaunay.h"

#include <cassert>
#include <memory>
#include <vector>

class TerrainGeneration : public InputListener
{
	// bounding box for a random point cloud
	glm::vec3 pointCloudMin = glm::vec3(-40.0f, -2.0f, -40.0f);
	glm::vec3 pointCloudMax = glm::vec3(40.0f, 2.0f, 40.0f);

public:
	TerrainGeneration() 
	{
		Init();
	}

	~TerrainGeneration() 
	{
		Terminate();
	}

	void OnKeyPressed(int key) override 
	{
		switch(key)
		{
			case GLFW_KEY_F1:
				ToggleWireframeMode();
				break;
			case GLFW_KEY_TAB:
				camera.PrintAttributes();
				break;
			case GLFW_KEY_0:
				pointCloud.CreateCustomTest();
				delaunay.Clear();
				break;
			case GLFW_KEY_1:
				pointCloud.CreateRandom(pointCloudMin, pointCloudMax);
				delaunay.Clear();
				break;
			case GLFW_KEY_2:
			{
				delaunay.Triangulate(pointCloud);
				break;
			}
			case GLFW_KEY_3:
			{
				delaunay.TriangulateByIterations(pointCloud);
				break;
			}
			case GLFW_KEY_4:
			{
				static unsigned n = 0;
				delaunay.ExportTriangulation(std::string("DelaunayTriangulation_") + std::to_string(n++));
				break;
			}
			case GLFW_KEY_5:
			{
				Mesh m;
				m.LoadWavefrontObj(std::string("assets/Triangulations/DelaunayTriangulation_0.obj"));
				break;
			}
			default:
				break;
		}
	};

	void OnKeyReleased(int key) override { };
	void OnMouseButtonPressed(int button, double x, double y) override { };
	void OnMouseButtonReleased(int button, double x, double y) override { };
	void OnMouseScroll(double xoffset, double yoffset) override { };
	
	void OnMouseMove(double x, double y) 
	{
	}

	void Update(float deltaTime) 
	{
		camera.Update(deltaTime);
	}

	void Render() 
	{
		DrawCubes();
		DrawPointCloud();
		DrawDelaunay();
	}
	
protected:

	void Init()
	{
		// init the vertex buffer object
		InitVBO();

		// init the indeces buffer object
		InitIBO();

		// init the vertex array object
		InitVAO();

		// load shader
		shader.Load("assets/Shaders/basic.vert", "assets/Shaders/basic.frag");

		// init cubes
		InitCubes();

		// init camera
		//camera.Init(glm::vec3(0.0f, 250.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), 45.0f, 1024.0f / 768.0f, 0.1f, 1000000.0f);
		//camera.Rotate(glm::vec3(-1.5708f, 0.0f, 0.0f));

		camera.Init(glm::vec3(1.0f, 4.0f, -120.0f), glm::vec3(0.0f, 0.0f, 0.0f), 45.0f, 1024.0f / 768.0f, 0.1f, 1000000.0f);
		camera.Rotate(glm::vec3(0.0f, 0.0f, 0.0f));
	}
		
	void InitVBO()
	{
		// create one buffer in the GPU, use it as an array buffer and set the data
		glGenBuffers(1, &vertexBufferObject);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	}

	void InitIBO()
	{
		// create one buffer in the GPU, use it as an element array buffer and set the data
		glGenBuffers(1, &indexBufferObject);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	}

	void InitVAO()
	{
		// create one vertex array object for drawing and use it. This vertexArrayObject is the one who will deal with the vertex buffer
		glGenVertexArrays(1, &vertexArrayObject);
		glBindVertexArray(vertexArrayObject);

		// Tell the vertex shader how to interpret the buffer data. This information is needed for the active vertexArrayObject
		// The 0 attribute(pos) has 3 elements (x,y,z) of type GL_FLOAT
		// The stride to the next 0 attribute is zero bytes because there are no other attributes in between
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

		// Enable the 0 attribute
		glEnableVertexAttribArray(0);
	}

	void InitCubes()
	{
		// floor
		//cubes[0].pos = glm::vec3(0.0f, 0.0f, 10.0f);
		//cubes[0].scale = glm::vec3(40.0f, 0.0001f, 40.f);
		//cubes[0].color = glm::vec4(0.8f, 0.8f, 0.8f, 1.0f);
		//cubes[0].enabled = true;
	}

	void DrawCubes()
	{
		const glm::mat4& viewProjection = camera.ViewProjectionMatrix();

		// use the shader
		shader.Use();	

		// tell the vertexArrayObject to be used
		glBindVertexArray(vertexArrayObject);

		glm::mat4 model;
		for (int i = 0; i < NUM_CUBES; i++)
		{
			Cube cube = cubes[i];
			if (cube.enabled)
			{
				model = glm::mat4();
				model = glm::translate(model, cube.pos) 
					  * glm::rotate(model, cube.rotation.z, glm::vec3(0.0f, 0.0f, 1.0f)) 
					  * glm::rotate(model, cube.rotation.y, glm::vec3(0.0f, 1.0f, 0.0f)) 
					  * glm::rotate(model, cube.rotation.x, glm::vec3(1.0f, 0.0f, 0.0f))
					  * glm::scale(model, cube.scale);
				shader.SetUniform("modelViewProjection", viewProjection * model);
				shader.SetUniform("color", cube.color);
				glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (void*)indices); // tell to draw cube by using the IBO
			}
		}

		// do not use the vertexArrayObject anymore
		glBindVertexArray(0);
	}

	void DrawPointCloud()
	{
		const glm::mat4& viewProjection = camera.ViewProjectionMatrix();

		// use the shader
		shader.Use();

		// tell the vertexArrayObject to be used
		glBindVertexArray(vertexArrayObject);

		auto& points = pointCloud.Points();

		glm::mat4 model;
		for (auto& point : points)
		{	
			model = glm::mat4();
			model = glm::translate(model, point) * glm::scale(model, glm::vec3(0.05f, 0.05f, 0.05f));
			shader.SetUniform("modelViewProjection", viewProjection * model);
			shader.SetUniform("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

			glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (void*)indices); // tell to draw cube by using the IBO
		}

		// do not use the vertexArrayObject anymore
		glBindVertexArray(0);
	}

	void DrawDelaunay()
	{
		// render DelaunayTriangle leafs
		DelaunayTriangle* rootTriangle = delaunay.RootTriangle();
		if (rootTriangle)
		{
			const glm::mat4& viewProjection = camera.ViewProjectionMatrix();

			auto& triangulation = delaunay.Triangulation();
			if (triangulation.size() > 0)
			{
				for (auto& triangle : triangulation)
				{
					DrawDelaunayTriangle(*triangle, viewProjection, shader);
				}
			}
			else
			{
				// draw recursively
				auto& triangle = *rootTriangle;
				DrawDelaunayTriangleLeafs(*rootTriangle, viewProjection, shader);
			}
		}
	}

	void DrawDelaunayTriangleLeafs(const DelaunayTriangle& triangle, const glm::mat4& viewProjection, Shader& shader)
	{
		auto& children = triangle.children;
		if (children.size() == 0)
		{
			DrawDelaunayTriangle(triangle, viewProjection, shader);
		}
		else
		{
			for (size_t i = 0; i < children.size(); i++)
			{
				DrawDelaunayTriangleLeafs(*children[i], viewProjection, shader);
			}
		}
	}

	void DrawDelaunayTriangle(const DelaunayTriangle& triangle, const glm::mat4& viewProjection, Shader& shader)
	{
		const glm::vec3& v1 = triangle.edge->v->v;
		const glm::vec3& v2 = triangle.edge->next->v->v;
		const glm::vec3& v3 = triangle.edge->next->next->v->v;

		// use the shader
		shader.Use();

		// tell the vertexArrayObject to be used
		glBindVertexArray(vertexArrayObject);
		
		glm::mat4 model;

		model = glm::mat4();
		model = glm::translate(model, v1) * glm::scale(model, glm::vec3(0.1f, 0.1f, 0.1f));
		shader.SetUniform("modelViewProjection", viewProjection * model);
		shader.SetUniform("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (void*)indices);

		model = glm::mat4();
		model = glm::translate(model, v2) * glm::scale(model, glm::vec3(0.1f, 0.1f, 0.1f));
		shader.SetUniform("modelViewProjection", viewProjection * model);
		shader.SetUniform("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (void*)indices);

		model = glm::mat4();
		model = glm::translate(model, v3) * glm::scale(model, glm::vec3(0.1f, 0.1f, 0.1f));
		shader.SetUniform("modelViewProjection", viewProjection * model);
		shader.SetUniform("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (void*)indices);
	
		
		shader.SetUniform("modelViewProjection", viewProjection);
		shader.SetUniform("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
		
		glBegin(GL_LINES);
		glVertex3f(v1.x, v1.y, v1.z);
		glVertex3f(v2.x, v2.y, v2.z);
		glVertex3f(v2.x, v2.y, v2.z);
		glVertex3f(v3.x, v3.y, v3.z);
		glVertex3f(v3.x, v3.y, v3.z);
		glVertex3f(v1.x, v1.y, v1.z);
		glEnd();

		// do not use the vertexArrayObject anymore
		glBindVertexArray(0);
	}

	void Terminate()
	{
		glDeleteVertexArrays(1, &vertexArrayObject);
		glDeleteBuffers(1, &vertexBufferObject);
		glDeleteBuffers(1, &indexBufferObject);
	}

	void ToggleWireframeMode()
	{
		wireframeMode = !wireframeMode;
		glPolygonMode(GL_FRONT_AND_BACK, wireframeMode ? GL_LINE : GL_FILL);		
	}

private:

	// vertices of the cube
	GLfloat vertices[24] =
	{
		-1.0f,	1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,	1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f
	};


	// indices to cube vertices
	GLuint indices[36] =
	{
	   // tri 1      tri 2
		0, 2, 1,	0, 3, 2,	// front face
		4, 6, 5,    4, 7, 6,	// back face	
		4, 3, 0,    4, 7, 3,	// left face	
		1, 6, 5,    1, 2, 6,	// right face	
		4, 1, 5,    4, 0, 1,	// top face	
		7, 2, 6,    7, 3, 2		// bottom face	
	};
	
	// vbo, vao, ibo 
	GLuint vertexBufferObject;
	GLuint indexBufferObject;
	GLuint vertexArrayObject; 

	// shader
	Shader shader;

	// cubes
	struct Cube
	{
		bool enabled = false;

		glm::vec3 pos = glm::vec3(0.0f, 0.0f, 0.0f);
		glm::vec3 rotation = glm::vec3(0.0f, 0.0f, 0.0f);
		glm::vec3 scale = glm::vec3(1.0f, 1.0f, 1.0f);
		glm::vec4 color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
	};

	static const int NUM_CUBES = 64;
	Cube cubes[NUM_CUBES];

	// camera
	FreeCamera camera;

	bool wireframeMode = false;

	// point cloud
	PointCloud pointCloud;

	// Delaunay triangulation
	Delaunay delaunay;
};

#endif