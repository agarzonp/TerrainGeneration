#ifndef TEXTURE_2D_H
#define TEXTURE_2D_H

#include <string>

#include "GL/glew.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image/stb_image.h"

class Texture2D
{
	// width, height, components
	int width = -1;
	int height = -1;
	int components = -1;

	// pixels
	unsigned char* pixels = nullptr;

public:
	Texture2D() {};
	~Texture2D() {};

	// Load
	bool Load(const std::string& filename) 
	{
		pixels = stbi_load(filename.c_str(), &width, &height, &components, STBI_rgb_alpha);
		if (!pixels)
		{
			printf("Unable to load texture %s", filename.c_str());
			return false;
		}

		return true;
	}

	// getters
	int GetWidth() { return width; }
	int GetHeight() { return height; }
	const unsigned char* GetPixels() const { return pixels; }

private:

};

#endif // !TEXTURE_2D_H

