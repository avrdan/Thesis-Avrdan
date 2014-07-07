#pragma once

#include <Windows.h>
#include "util_pipeline.h"
#include "util_render.h"
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

// include GLM math library
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

#include "pxcprojection.h"
#include "pxcmetadata.h"
#include "pxcsegmentation.h"

#include <ppl.h>

class RawDepthPipeline
{
public:
	RawDepthPipeline();
	~RawDepthPipeline();

	static const int depthCamWidth = 320;
	static const int depthCamHeight = 240;
	static const int rgbCamWidth = 640;
	static const int rgbCamHeight = 480;
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
		std::vector<unsigned int> indices;

		
		//glm::vec3 positions[];
		//glm::vec2 uvs[];
	};
	sVertexType* pVertexMem;

	pxcUID pid;
	PXCProjection *projection;
	pxcU32 nPoints;
	pxcU32 nPointsRGB;
	PXCPoint3DF32* pos3dDepth;
	PXCPoint3DF32* pos2d;
	PXCPointF32* pos2dColor;
	PXCPoint3DF32* pos3d;
	std::vector<unsigned int> indices;
	std::vector<unsigned short> depthData;

	void renderFrame(bool useMedianFiltering, bool useWeightedMovingAverage);
	pcl::PointCloud<pcl::PointXYZ>::Ptr renderFramePCL();
	std::vector<PXCPoint3DF32> worldPos;
	std::vector<PXCPoint3DF32> screenPos;

	float centerDepth;
private:

	PXCCapture::VideoStream::ProfileInfo pinfo;
	PXCSegmentation::ProfileInfo pSegInfo;
	UtilPipeline pp;
	//UtilRender depth_render;
	UtilRender *depth_render;
	UtilRender *color_render;
	// special depth values for saturated and low-confidence pixels
	pxcF32 dvalues[2];
	bool g_stop;
	pxcStatus status;

	/* Segmentation algorithm parameter */
	pxcCHAR* g_bg_image_file;
	int g_max_distance;
	PXCSegmentation::BlendMode g_blend_mode;
	PXCSegmentation::FaceMode  g_face_mode;
	PXCSegmentation *segmentation;

	int numStripsRequired;
	int numDegensRequired;
	int verticesPerStrip;

	int nSize;

	void printPointCloudData();
	void printSelectivePointCloudData(int selector);
	void createPointCloud(PXCImage::ImageData ddepth);
	void createPointCloudMappedToWorld(PXCImage::ImageData ddepth, bool useMedianFiltering, bool useWeightedMovingAverage);
	void createPointCloudMappedToWorld(PXCImage::ImageData ddepth, PXCImage::ImageData dcolor);
	//void addIndexData();
	void addIndexData(int width, int height);
	void addIndexDataTriangles();
	void addIndexDataTriangleStrip();
	pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloudPCL(PXCImage::ImageData ddepth);
	void renderLoop();

	// median filter window size
	static const int windowSize = 9;
	//static const int windowSize = 25;
	void insertionSort(int window[]);
	PXCPoint3DF32* medianFilter();
	//create a sliding window of size 9
	int window[windowSize];
	PXCPoint3DF32* newPos2d;

	// weighted moving average
	std::list<PXCPoint3DF32*> averageList;
	PXCPoint3DF32* weightedMovingAverage();
	void checkForDequeue();
	int averageFrameCount;
	pxcF32* sumDepthArray;
	PXCPoint3DF32* averagedDepthArray;
};

