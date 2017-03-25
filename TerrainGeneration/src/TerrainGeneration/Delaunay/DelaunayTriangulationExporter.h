#ifndef DELAUNAY_TRIANGULATION_EXPORTER
#define DELAUNAY_TRIANGULATION_EXPORTER

#include "DelaunayStructures.h"

enum class TriangulationExportFormat
{
	WAVEFRONT_OBJ
};

class DelaunayTriangulationExporter
{
public:
	DelaunayTriangulationExporter() {};
	~DelaunayTriangulationExporter() {};

	void Export(TriangulationExportFormat format, std::vector<DelaunayTriangle*>& triangulation)
	{
		switch (format)
		{
		case TriangulationExportFormat::WAVEFRONT_OBJ:
			ExportToWavefrontObj(triangulation);
			break;
		}
	}

private:

	void ExportToWavefrontObj(std::vector<DelaunayTriangle*>& triangulation)
	{

	}
};

#endif // !TRIANGULATION_EXPORTER

