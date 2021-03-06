#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H

#include <vector>

#include "../../Texture2D/Texture2D.h"

class HeightMap
{
	// heights
	std::vector<float> heights;

	// width and depth
	int width = -1;
	int depth = -1;

public:
	HeightMap() {};
	~HeightMap() {};

	// Load
	bool Load(const std::string& filename)
	{
		// load the texture
		Texture2D texture;
		bool loaded = texture.Load(filename);
		if (loaded)
		{
			// set width and depth
			width = texture.GetWidth();
			depth = texture.GetHeight();

			float maxtHeight = 10.0f;

			// get the pixels and set the heights
			const unsigned char* pixels = texture.GetPixels();
			for (int w = 0; w < width; w++)
			{
				for (int d = 0; d < depth; d++)
				{
					// use the red colour for the heightmap
					unsigned char r = pixels[(w + d * width) * 4 ];
					float h = maxtHeight * ((r / 255.0f) - 0.5f);
					heights.push_back(h);
				}
			}
		}
				
		return loaded;
	}

	// Getters
	int Width() { return width; }
	int Depth() { return depth; }

	float Height(int x, int y)
	{
		unsigned index = x * depth + y;
		if (index >= heights.size())
		{
			assert(false);
			return -1;
		}

		return heights[index];
	}
private:

};

#endif // !HEIGHTMAP_H

