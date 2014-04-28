#pragma once

#include <Windows.h>
#include "util_pipeline.h"
#include "util_render.h"
#include <iostream>

// include GLM math library
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

class RawDepthPipeline
{
public:
	RawDepthPipeline();
	~RawDepthPipeline();

	static const int depthCamWidth = 320;
	static const int depthCamHeight = 240;

	// data structure for mesh
	struct sVertexType
	{
		float fX;
		float fY;
		float fZ;
		float fNX;
		float fNY;
		float fU;
		float fV;
		//glm::vec3 pos;
		//glm::vec2 uv;
		//glm::vec3 normal;
	
		std::vector<glm::vec3> positions;
		std::vector<glm::vec2> uvs;
		std::vector<glm::vec3> normals;
		//glm::vec3 positions[];
		//glm::vec2 uvs[];
	};
	sVertexType* pVertexMem;

	void renderFrame();
private:

	PXCCapture::VideoStream::ProfileInfo pinfo;
	UtilPipeline pp;
	//UtilRender depth_render;
	UtilRender *depth_render;

	void printPointCloudData();
	void printSelectivePointCloudData(int selector);
	void createPointCloud(PXCImage::ImageData ddepth);
	void renderLoop();
};

