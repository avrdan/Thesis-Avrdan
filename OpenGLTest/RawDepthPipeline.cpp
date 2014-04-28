#include "RawDepthPipeline.h"


RawDepthPipeline::RawDepthPipeline()
{
	// init data structure
	pVertexMem = new sVertexType[depthCamWidth * depthCamHeight];

	pp.EnableImage(PXCImage::COLOR_FORMAT_DEPTH);
	pp.Init();
	//UtilRender depth_render(L"Depth Stream");
	//depth_render = UtilRender(L"Depth Stream");
	depth_render = new UtilRender(L"Depth Stream");

	// set framerate to 60 fps
	pp.QueryCapture()->QueryVideoStream(0)->QueryProfile(&pinfo);
	pinfo.frameRateMin.numerator = 60;
	pinfo.frameRateMin.denominator = 1;
	pinfo.frameRateMax = pinfo.frameRateMin;
	pp.QueryCapture()->QueryVideoStream(0)->SetProfile(&pinfo);


	pVertexMem->positions.reserve(depthCamWidth * depthCamHeight);
	pVertexMem->uvs.reserve(depthCamWidth * depthCamHeight);
	// start depth rendering loop
	// and frame acquisition
	//renderLoop();
}


RawDepthPipeline::~RawDepthPipeline()
{
	pp.Close();
}

void RawDepthPipeline::renderLoop()
{
	for (;;)
	{
		if (!pp.AcquireFrame(true)) break;
		//PXCImage *color_image = pp.QueryImage(PXCImage::IMAGE_TYPE_COLOR);
		PXCImage *depth_image = pp.QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
		//if (!color_render.RenderFrame(color_image)) break;
		if (!depth_render->RenderFrame(depth_image)) break;

		// get depth data
		PXCImage::ImageData ddepth;

		depth_image->AcquireAccess(PXCImage::ACCESS_READ, &ddepth);

		createPointCloud(ddepth);
		printPointCloudData();
		// NOTE THAT NOT ALL ARRAY VALUES ARE VALID FOR
		// JUST THE ONES WITH DEPTH > 10
		//printPointCloudData();
		//printSelectivePointCloudData(100);

		depth_image->ReleaseAccess(&ddepth);
		pp.ReleaseFrame();
	}
}

void RawDepthPipeline::renderFrame()
{
	if (!pp.AcquireFrame(true)) return;
	//PXCImage *color_image = pp.QueryImage(PXCImage::IMAGE_TYPE_COLOR);
	PXCImage *depth_image = pp.QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
	//if (!color_render.RenderFrame(color_image)) break;
	if (!depth_render->RenderFrame(depth_image)) return;

	// get depth data
	PXCImage::ImageData ddepth;

	depth_image->AcquireAccess(PXCImage::ACCESS_READ, &ddepth);

	createPointCloud(ddepth);
	// NOTE THAT NOT ALL ARRAY VALUES ARE VALID FOR
	// JUST THE ONES WITH DEPTH > 10
	//printPointCloudData();
	//printSelectivePointCloudData(100);

	depth_image->ReleaseAccess(&ddepth);
	pp.ReleaseFrame();
}

void RawDepthPipeline::printSelectivePointCloudData(int selector)
{
	for (int n = 0; n < depthCamWidth * depthCamHeight; n++)
	{
		if (n % selector == 0)
		{
			std::cout << "Vertex " << n << ": (" << pVertexMem[n].fX << ", " <<
				pVertexMem[n].fY << ", " << pVertexMem[n].fZ << ")" << std::endl;
			//std::cout << "UV " << n << ": (" << pVertexMem[n].fU << ", " << pVertexMem[n].fV << ")" << std::endl;
		}
	}
}

void RawDepthPipeline::printPointCloudData()
{
	/*for (int n = 0; n < depthCamWidth * depthCamHeight; n++)
	{
		std::cout << "Vertex " << n << ": (" << pVertexMem[n].fX << ", " <<
			pVertexMem[n].fY << ", " << pVertexMem[n].fZ << ")" << std::endl;
	}*/
	for (std::vector<glm::vec3>::const_iterator i = pVertexMem->positions.begin(); i != pVertexMem->positions.end(); ++i)
	{
		std::cout << "(" << i->x << ", " << i->y << ", " << i->z << ")" << std::endl;
	}
		
}

void RawDepthPipeline::createPointCloud(PXCImage::ImageData ddepth)
{
	int n = 0;
	pVertexMem->positions.clear();
	//pVertexMem->uvs.clear();

	for (int y = 0; y < depthCamHeight; y++)
	{
		for (int x = 0; x < depthCamWidth; x++)
		{
			float fX = (float)x / 100.0f;
			float fY = (float)y / 100.0f;
			float fZ = 0.0f;
			float fU = 0.0f;
			float fV = 0.0f;
			// data.planes[0][y*cols+x];
			pxcU16 depthValue = ((pxcU16*)ddepth.planes[0])[y*pinfo.imageInfo.width + x];

			if (depthValue && depthValue < 1500)
			{
				fZ = (((depthValue - 10) / 10.0f ) / 100.0f);
				fU = fX / (float)depthCamWidth;
				fV = fY / (float)depthCamHeight;

				pVertexMem->positions.push_back(glm::vec3(fX, fY, fZ));
			}
				// put this in outer loop
				// if all values of 320*240
				// are to be populated
				/*pVertexMem[n].fX = fX;
				pVertexMem[n].fY = fY;
				pVertexMem[n].fZ = fZ;
				pVertexMem[n].fU = fU;
				pVertexMem[n].fV = fV;*/

				//pVertexMem[n].pos = glm::vec3(fX, fY, fZ);
				//pVertexMem[n].uv  = glm::vec2(fU, fV);

				//pVertexMem->positions[n] = glm::vec3(fX, fY, fZ);
				//pVertexMem->uvs[n]		 = glm::vec2(fU, fV);
		
				//pVertexMem->uvs.push_back(glm::vec2(fU, fV));

				n++;
		}
	}
}