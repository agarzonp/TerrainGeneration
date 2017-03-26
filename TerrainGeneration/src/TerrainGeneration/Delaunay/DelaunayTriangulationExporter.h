#ifndef DELAUNAY_TRIANGULATION_EXPORTER
#define DELAUNAY_TRIANGULATION_EXPORTER

#include "DelaunayStructures.h"

#include <fstream>

enum class TriangulationExportFormat
{
	WAVEFRONT_OBJ
};

class DelaunayTriangulationExporter
{
public:
	DelaunayTriangulationExporter() {};
	~DelaunayTriangulationExporter() {};

	// Export
	void Export(TriangulationExportFormat format, std::vector<DelaunayTriangle*>& triangulation, std::string& filename, bool registerNewFile)
	{
		switch (format)
		{
		case TriangulationExportFormat::WAVEFRONT_OBJ:
			ExportToWavefrontObj(triangulation, filename, registerNewFile);
			break;
		}
	}

private:

	// Export to Wavefront .obj
	void ExportToWavefrontObj(std::vector<DelaunayTriangle*>& triangulation, std::string& filename, bool registerNewFile)
	{
		std::string _filename = "assets/Triangulations/" + filename + ".obj";
		std::ofstream file(_filename);
		if (!file.is_open())
		{
			return;			
		}
		
		file << "# Delaunay triangulation" << std::endl << std::endl; 

		file << "# vertices (x, y, z)" << std::endl << std::endl;

		unsigned numVertices = 0;
		for (auto& triangle : triangulation)
		{
			DelaunayVertex* v1 = triangle->edge->v;
			DelaunayVertex* v2 = triangle->edge->next->v;
			DelaunayVertex* v3 = triangle->edge->next->next->v;

			ExportVertexToWavefronObj(file, v1, numVertices);
			ExportVertexToWavefronObj(file, v2, numVertices);
			ExportVertexToWavefronObj(file, v3, numVertices);
		}

		file << std::endl << "# faces" << std::endl << std::endl;

		for (auto& triangle : triangulation)
		{
			ExportFaceToWavefronObj(file, triangle);
		}

		file.close();

		// register new file
		if (registerNewFile)
		{
			std::ofstream outfile;

			outfile.open("assets/Triangulations/triangulations.txt", std::ios_base::app);
			if (outfile)
			{
				outfile << std::endl << _filename;
			}
		}
	}

	// Export vertex Wavefront .obj
	void ExportVertexToWavefronObj(std::ofstream& file, DelaunayVertex* vertex, unsigned& numVertices)
	{
		if (vertex->exportIndex == -1)
		{
			vertex->exportIndex = numVertices++;

			const glm::vec3& v = vertex->v;

			file << "v " << v.x << " " << v.y << " " << v.z << std::endl;
		}
	}

	// Export face to Wavefront .obj
	void ExportFaceToWavefronObj(std::ofstream& file, DelaunayTriangle* triangle)
	{
		DelaunayVertex* v1 = triangle->edge->v;
		DelaunayVertex* v2 = triangle->edge->next->v;
		DelaunayVertex* v3 = triangle->edge->next->next->v;

		file << "f " << v1->exportIndex << " " << v2->exportIndex << " " << v3->exportIndex << std::endl;
	}
};

#endif // !TRIANGULATION_EXPORTER

