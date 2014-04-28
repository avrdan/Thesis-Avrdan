#include "texture.h"
#include <iostream>

Texture::Texture()
{
}


Texture::~Texture()
{
}

bool Texture::loadTexture2D(string a_sPath, bool bGenerateMipMaps)
{
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	FIBITMAP* dib(0);

	fif = FreeImage_GetFileType(a_sPath.c_str(), 0); // check file signature and format

	if (fif == FIF_UNKNOWN) // try to guess
	{
		fif = FreeImage_GetFIFFromFilename(a_sPath.c_str());
	}

	if (fif == FIF_UNKNOWN) // failure
	{
		return false;
	}

	if (FreeImage_FIFSupportsReading(fif)) // if we can read the file, load it
	{
		dib = FreeImage_Load(fif, a_sPath.c_str());
	}

	if (!dib)
		return false;

	BYTE* bDataPointer = FreeImage_GetBits(dib); // retrieve the image data

	iWidth  = FreeImage_GetWidth(dib);
	iHeight = FreeImage_GetHeight(dib);
	iBPP = FreeImage_GetBPP(dib);
	
	// if anything is wrong, return
	if (bDataPointer == NULL || iWidth == 0 || iHeight == 0)
		return false;

	// Generate an OpenGL texture ID for this texture
	glGenTextures(1, &uiTexture);
	glBindTexture(GL_TEXTURE_2D, uiTexture);

	int iFormat = iBPP == 24 ? GL_BGR : iBPP == 8 ? GL_LUMINANCE : 0;
	int iInternalFormat = iBPP == 24 ? GL_RGB : GL_DEPTH_COMPONENT;

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, iWidth, iHeight, 0, iFormat, GL_UNSIGNED_BYTE, bDataPointer);

	// clamp textures
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	if (bGenerateMipMaps)
		glGenerateMipmap(GL_TEXTURE_2D);
	FreeImage_Unload(dib);

	glGenSamplers(1, &uiSampler);

	sPath = a_sPath;
	bMipMapsGenerated = bGenerateMipMaps;

	return true; // Success
}

void Texture::setSamplerParameter(GLenum parameter, GLenum value)
{
	glSamplerParameteri(uiSampler, parameter, value);
}

void Texture::setFiltering(int a_tfMagnification, int a_tfMinification)
{
	// set magnification filter
	if (a_tfMagnification == TEXTURE_FILTER_MAG_NEAREST)
	{
		glSamplerParameteri(uiSampler, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	else if (a_tfMagnification == TEXTURE_FILTER_MAG_BILINEAR)
	{
		glSamplerParameteri(uiSampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}

	// set minification filter
	if (a_tfMinification == TEXTURE_FILTER_MIN_NEAREST)
	{
		glSamplerParameteri(uiSampler, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	}
	else if (a_tfMinification == TEXTURE_FILTER_MIN_BILINEAR)
	{
		glSamplerParameteri(uiSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	}
	else if (a_tfMinification == TEXTURE_FILTER_MIN_NEAREST_MIPMAP)
	{
		glSamplerParameteri(uiSampler, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	}
	else if (a_tfMinification == TEXTURE_FILTER_MIN_BILINEAR_MIPMAP)
	{
		glSamplerParameteri(uiSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	}
	else if (a_tfMinification == TEXTURE_FILTER_MIN_TRILINEAR)
	{
		glSamplerParameteri(uiSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	}

	tfMinification  = a_tfMinification;
	tfMagnification = a_tfMagnification;
}

int Texture::getMinificationFilter()
{
	return tfMinification;
}

int Texture::getMagnificationFilter()
{
	return tfMagnification;
}

void Texture::bindTexture(int iTextureUnit)
{
	glActiveTexture(GL_TEXTURE0 + iTextureUnit);

	glBindTexture(GL_TEXTURE_2D, uiTexture);
	glBindSampler(iTextureUnit, uiSampler);
}

void Texture::releaseTexture()
{
	glDeleteSamplers(1, &uiSampler);
	glDeleteTextures(1, &uiTexture);
}

int Texture::getWidth()
{
	return iWidth;
}

int Texture::getHeight()
{
	return iHeight;
}

int Texture::getBPP()
{
	return iBPP;
}

GLuint Texture::getTextureID()
{
	return uiTexture;
}

GLuint Texture::getSamplerID()
{
	return uiSampler;
}

string Texture::getPath()
{
	return sPath;
}

//Texture tTextures[];
vector<Texture> tTextures = vector<Texture>();
int Texture::NUMTEXTURES = 0;

void loadAllTextures(vector<string> sTextureNames)
{
	Texture::NUMTEXTURES = sTextureNames.size();
	
	// Load textures

	// hard coded
	//string sTextureNames[] = { "sand_grass_02.jpg" };
	//NUMTEXTURES = sTextureNames.size();

	for (int i = 0; i < Texture::NUMTEXTURES; i++)
	{
		Texture *tex = new Texture();
		

		bool t = tex->loadTexture2D("./assets/textures/" + sTextureNames[i], true);
		
		if (!t)
		{
			cout << "Error loading texture: " << "./assets/textures/" + sTextureNames[i] << endl;
			continue;
		}

		cout << "Loaded Texture " << sTextureNames[i] << endl;

		tex->setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_BILINEAR_MIPMAP);
		tTextures.push_back(*tex);
	}
}
