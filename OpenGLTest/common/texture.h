#pragma once

#include <string>
#include <GL/glew.h>
// image handling
#include <FreeImage.h>
#include <vector>
using namespace std;

enum ETextureFiltering
{
	TEXTURE_FILTER_MAG_NEAREST = 0, // Nearest criterion for magnification
	TEXTURE_FILTER_MAG_BILINEAR, // Bilinear criterion for magnification
	TEXTURE_FILTER_MIN_NEAREST, // Nearest criterion for minification
	TEXTURE_FILTER_MIN_BILINEAR, // Bilinear criterion for minification
	TEXTURE_FILTER_MIN_NEAREST_MIPMAP, // Nearest criterion for minification, but on closest mipmap
	TEXTURE_FILTER_MIN_BILINEAR_MIPMAP, // Bilinear criterion for minification, but on closest mipmap
	TEXTURE_FILTER_MIN_TRILINEAR, // Bilinear criterion for minification on two closest mipmaps, then averaged
};

class Texture
{
public:
	Texture();
	~Texture();

	bool loadTexture2D(string a_sPath, bool generateMipMaps = false);
	void bindTexture(int iTextureUnit = 0);

	void setFiltering(int a_tfMagnification, int a_tfMinification);

	void setSamplerParameter(GLenum parameter, GLenum value);

	int getMinificationFilter();
	int getMagnificationFilter();

	int getWidth();
	int getHeight();
	int getBPP();

	GLuint getTextureID();
	GLuint getSamplerID();
	string getPath();

	void releaseTexture();
	static int NUMTEXTURES;
private:
	int iWidth, iHeight, iBPP;	// texture width, height, bytes per pixel
	GLuint uiTexture; // tex name
	GLuint uiSampler; // sampler name
	bool bMipMapsGenerated;

	int tfMinification, tfMagnification;

	string sPath;
};

//#define NUMTEXTURES 1
//extern Texture tTextures[];
extern vector<Texture> tTextures;
void loadAllTextures(vector<string> sTextureNames);