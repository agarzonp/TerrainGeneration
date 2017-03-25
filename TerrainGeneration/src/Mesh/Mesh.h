#ifndef MESH_H
#define MESH_H

#include "glm/glm.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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

public:
	Mesh() {};
	~Mesh() {};

	bool LoadWavefrontObj(std::string& filename)
	{
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

		return true;
	}

private:

};

#endif // !MESH_H

