#ifndef MESH_H
#define MESH_H

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "glm/glm.hpp"
#include "GL/glew.h"

#include "../Shaders/Shader.h"

struct MeshVertex
{
	glm::vec3 pos;
};

class Mesh
{
	// vertices
	std::vector<MeshVertex> vertices;

	// indices
	std::vector<GLuint> indices;

	// vbo, vao, ibo 
	GLuint vertexBufferObject;
	GLuint indexBufferObject;
	GLuint vertexArrayObject;

public:
	Mesh() {};
	~Mesh()
	{
		Clear();
	};

	// Clear
	void Clear()
	{
		if (vertices.size())
		{
			glDeleteVertexArrays(1, &vertexArrayObject);
			glDeleteBuffers(1, &vertexBufferObject);
			glDeleteBuffers(1, &indexBufferObject);

			vertices.clear();
			indices.clear();
		}
	}

	// num vertices
	size_t NumVertices() { return vertices.size(); }

	// Load mesh from Wavefront .obj
	bool LoadWavefrontObj(std::string& filename)
	{
		// clear current mesh
		Clear();

		std::ifstream file(filename);
		if (!file)
		{
			printf("%s Not Loaded", filename.c_str());
			return false;
		}

		// parse 
		std::string line;
		while (std::getline(file, line))
		{
			if (line.substr(0, 2) == "v ")
			{
				// new vertex
				std::istringstream iss(line.substr(2));

				MeshVertex vertex;
				iss >> vertex.pos.x;
				iss >> vertex.pos.y;
				iss >> vertex.pos.z;

				vertices.push_back(vertex);
			}
			else if (line.substr(0, 2) == "f ")
			{
				// new face
				std::istringstream iss(line.substr(2));

				GLuint index;
				iss >> index;
				indices.push_back(index);
				iss >> index;
				indices.push_back(index);
				iss >> index;
				indices.push_back(index);
			}
		}

		file.close();

		// init buffers
		InitBuffers();

		return true;
	}

	// Draw
	void Draw(const glm::mat4& viewProjection, Shader& shader)
	{
		if (vertices.size() > 0)
		{
			// use the shader
			shader.Use();

			// tell the vertexArrayObject to be used
			glBindVertexArray(vertexArrayObject);

			shader.SetUniform("modelViewProjection", viewProjection);
			shader.SetUniform("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

			// draw using the indices
			glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, (void*)(&indices[0]));

			// do not use the vertexArrayObject anymore
			glBindVertexArray(0);
		}
	}

private:

	// init buffers
	void InitBuffers()
	{
		if (vertices.size() == 0)
		{
			return;
		}

		// init the vertex buffer object
		InitVBO();

		// init the indeces buffer object
		InitIBO();

		// init the vertex array object
		InitVAO();
	}

	void InitVBO()
	{
		// create one buffer in the GPU, use it as an array buffer and set the data
		glGenBuffers(1, &vertexBufferObject);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MeshVertex), &vertices[0], GL_STATIC_DRAW);
	}

	void InitIBO()
	{
		// create one buffer in the GPU, use it as an element array buffer and set the data
		glGenBuffers(1, &indexBufferObject);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
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

		glBindVertexArray(0);
	}

};

#endif // !MESH_H

