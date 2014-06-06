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
	
	pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_SMOOTHING, true);
	//pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_SENSOR_RANGE, new float[]{200, 1000});
	//pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE, 500);
	//pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE, 100);
	// set framerate to 60 fps
	pp.QueryCapture()->QueryVideoStream(0)->QueryProfile(&pinfo);
	pinfo.frameRateMin.numerator = 60;
	pinfo.frameRateMin.denominator = 1;
	pinfo.frameRateMax = pinfo.frameRateMin;
	

	pp.QueryCapture()->QueryVideoStream(0)->SetProfile(&pinfo);
	
	pp.QueryCapture()->QueryDevice()->SetProperty(PXCCapture::Device::PROPERTY_DEPTH_SMOOTHING, 1);
	//pp.QueryCapture()->QueryDevice()->SetProperty(PXCCapture::Device::PROPERTY_DEPTH_, 1);
	//pp.QueryCapture()->QueryDevice()->SetProperty(PXCCapture::Device::PROPERTY_DEPTH_CONFIDENCE_THRESHOLD, 300);
	//pp.QueryCapture()->QueryDevice()->QueryPropertyAsUID(PXCSegmentation::, &pid);
	/*
	g_max_distance = 850;
	g_blend_mode = PXCSegmentation::BLEND_ANY;
	g_face_mode = PXCSegmentation::FACE_ALG_DEFAULT;
//	pp.EnableSegmentation(segmentation->CUID);

	pSegInfo.faceMode = g_face_mode;
	pSegInfo.maxDepthThreshold = g_max_distance;

	status = pp.QuerySegmentation()->SetProfile(&pSegInfo);
	if (status<PXC_STATUS_NO_ERROR) {
		//SetStatus(hwndDlg, L"Failed to set module profile");
		cout << "Error initializing segmentation module" << endl;
		g_stop = true;
	}
	*/
	/* Retrieve the serializable meta data from the capture device */
	pp.QueryCapture()->QueryDevice()->QueryPropertyAsUID(PXCCapture::Device::PROPERTY_PROJECTION_SERIALIZABLE, &pid);
	/* Recreate the Projection instance */
	pp.QuerySession()->DynamicCast<PXCMetadata>()->CreateSerializable<PXCProjection>(pid, &projection);

	pVertexMem->positions.reserve(depthCamWidth * depthCamHeight);
	pVertexMem->uvs.reserve(depthCamWidth * depthCamHeight);
	

	//pVertexMem->indices.resize((verticesPerStrip * numStripsRequired) + numDegensRequired);

	nPoints = depthCamWidth*depthCamHeight;
	pos2d = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	pos3d = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	worldPos.reserve(nPoints);
	screenPos.reserve(nPoints);
	//screenPos.resize(nPoints);
	// start depth rendering loop
	// and frame acquisition
	//renderLoop();
	numStripsRequired = depthCamHeight - 1;
	numDegensRequired = 2 * (numStripsRequired - 1);
	verticesPerStrip = 2 * depthCamWidth;
	indices.reserve((verticesPerStrip * numStripsRequired) + numDegensRequired);
	//indices.reserve((verticesPerStrip * numStripsRequired) + numDegensRequired);
	//indices.reserve((depthCamHeight - 1) * (depthCamWidth - 1) * 6); // 2 triangles per grid square x 3 vertices per triangle

	addIndexData();
	//addIndexDataTriangles();
	//addIndexDataTriangleStrip();
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
		//createPointCloudMappedToWorld(ddepth);
		//printPointCloudData();
		// NOTE THAT NOT ALL ARRAY VALUES ARE VALID FOR
		// JUST THE ONES WITH DEPTH > 10
		//printPointCloudData();
		//printSelectivePointCloudData(100);

		depth_image->ReleaseAccess(&ddepth);
		//pp.ReleaseFrame();
	}
}

void RawDepthPipeline::renderFrame()
{
	/*if ((pSegInfo.faceMode != g_face_mode) || (pSegInfo.maxDepthThreshold != g_max_distance))
	{
		pSegInfo.faceMode = g_face_mode;
		pSegInfo.maxDepthThreshold = g_max_distance;
		status = pp.QuerySegmentation()->SetProfile(&pSegInfo);
		if (status<PXC_STATUS_NO_ERROR) {
			//SetStatus(hwndDlg, L"Failed to set module profile");
			return;
		}
	}*/


	if (!pp.AcquireFrame(true)) return;
	//PXCImage *color_image = pp.QueryImage(PXCImage::IMAGE_TYPE_COLOR);
	PXCImage *depth_image = pp.QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
	//PXCImage *depth_image = pp.QuerySegmentationImage();
	//if (!color_render.RenderFrame(color_image)) break;
	if (!depth_render->RenderFrame(depth_image)) return;

	// get depth data
	PXCImage::ImageData ddepth;

	depth_image->AcquireAccess(PXCImage::ACCESS_READ, &ddepth);

	//createPointCloud(ddepth);
	createPointCloudMappedToWorld(ddepth);
	// NOTE THAT NOT ALL ARRAY VALUES ARE VALID FOR
	// JUST THE ONES WITH DEPTH > 10
	//printPointCloudData();
	//printSelectivePointCloudData(100);

	depth_image->ReleaseAccess(&ddepth);
	pp.ReleaseFrame();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RawDepthPipeline::renderFramePCL()
{
	if (!pp.AcquireFrame(true)) return NULL;
	//PXCImage *color_image = pp.QueryImage(PXCImage::IMAGE_TYPE_COLOR);
	PXCImage *depth_image = pp.QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
	//if (!color_render.RenderFrame(color_image)) break;
	if (!depth_render->RenderFrame(depth_image)) return NULL;

	// get depth data
	PXCImage::ImageData ddepth;

	depth_image->AcquireAccess(PXCImage::ACCESS_READ, &ddepth);

	pcl::PointCloud<pcl::PointXYZ>::Ptr frameCloud = createPointCloudPCL(ddepth);
	// NOTE THAT NOT ALL ARRAY VALUES ARE VALID FOR
	// JUST THE ONES WITH DEPTH > 10
	//printPointCloudData();
	//printSelectivePointCloudData(100);

	depth_image->ReleaseAccess(&ddepth);
	pp.ReleaseFrame();

	return frameCloud;
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

void RawDepthPipeline::createPointCloudMappedToWorld(PXCImage::ImageData ddepth)
{
	worldPos.clear();
	//screenPos.clear();
	pos2d = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	//screenPos = std::vector<PXCPoint3DF32>(nPoints);
	
	//screenPos.erase(screenPos.begin(), screenPos.end());
	//screenPos.resize(nPoints);
	//screenPos.reserve(nPoints);
	//worldPos = new std::vector<PXCPoint3DF32>(nPoints);
	//delete[] pos3d;
	//pos3d = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];

	int n = 0;
	//pos3d = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	// find depth image stride
	int depthStride = ddepth.pitches[0] / sizeof(pxcU16);
	pxcU16 lastDepthValue = 0;
	int undefined;
	for (int y = 0; y < depthCamHeight; y++)
	{
		for (int x = 0; x < depthCamWidth; x++)
		{
			pxcU16 depthValue = ((pxcU16*)ddepth.planes[0])[y * depthStride + x];
			
			//screenPos[n].x = (pxcF32)x;
			//screenPos[n].y = (pxcF32)y;
			//screenPos[n].z = (pxcF32)1000;
			//pos2d[n].z = (pxcF32)1000;
			//pos2d[n].x = undefined;
			//pos2d[n].y = undefined;
			//pos2d[n].z = undefined;
			// raw depth data
			
			lastDepthValue = depthValue;
			if (depthValue > 10 && depthValue < 2000)
			{
				//screenPos[n].x = (pxcF32)x;
				//screenPos[n].y = (pxcF32)y;
				//screenPos[n].z = (pxcF32)depthValue;

				pos2d[n].x = (pxcF32)x;
				pos2d[n].y = (pxcF32)y;
				pos2d[n].z = (pxcF32)depthValue;
				
				
			}
			n++;
		}
	}

	//projection->ProjectImageToRealWorld(nPoints, screenPos.data(), worldPos.data());
	projection->ProjectImageToRealWorld(nPoints, &pos2d[0], worldPos.data());	

	// use this to align depth with color
	//projection->MapDepthToColorCoordinates

	delete[] pos2d;
	
	//indices.clear();
	//addIndexData();
	//std::copy(&pos3d[0], &pos3d[nPoints], std::back_inserter(worldPos));
}

void RawDepthPipeline::createPointCloud(PXCImage::ImageData ddepth)
{
	int n = 0;
	pVertexMem->positions.clear();
	//pVertexMem->indices.clear();
	//pVertexMem->uvs.clear();

	// find depth image stride
	int depthStride = ddepth.pitches[0] / sizeof(pxcU16);

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
				pVertexMem->positions.push_back(glm::vec3(fX, fY, fZ));

				n++;
		}
	}

	//addIndexData();
	/*for (int y = 0; y < depthCamHeight-1; ++y)
	{
		for (int x = 0; x < depthCamWidth-1; ++x)
		{
			int start = y * depthCamWidth + x;
			pVertexMem->indices.push_back((unsigned int)start);
			pVertexMem->indices.push_back((unsigned int)(start + 1));
			pVertexMem->indices.push_back((unsigned int)(start + depthCamWidth));
			pVertexMem->indices.push_back((unsigned int)(start + 1));
			pVertexMem->indices.push_back((unsigned int)(start + 1 + depthCamWidth));
			pVertexMem->indices.push_back((unsigned int)(start + depthCamWidth));
		}
	}*/

}

void RawDepthPipeline::addIndexDataTriangles()
{
	for (int y = 0; y < depthCamHeight - 1; ++y)
	{
		for (int x = 0; x < depthCamWidth - 1; ++x)
		{
			int start = y * depthCamWidth + x;

			indices.push_back((unsigned int)start);
			indices.push_back((unsigned int)(start + 1));
			indices.push_back((unsigned int)(start + depthCamWidth));
			indices.push_back((unsigned int)(start + 1));
			indices.push_back((unsigned int)(start + 1 + depthCamWidth));
			indices.push_back((unsigned int)(start + depthCamWidth));
		}
	}
}

void RawDepthPipeline::addIndexData()
{
	int n = 0;
	int colSteps = depthCamWidth * 2;
	int rowSteps = depthCamHeight - 1;
	for (int r = 0; r < rowSteps; r++) {
		for (int c = 0; c < colSteps; c++) {
			int t = c + r * colSteps;

			if (c == colSteps - 1) {
				indices.push_back(n);
			}
			else {
				indices.push_back(n);

				if (t % 2 == 0) {
					n += depthCamWidth;
				}
				else {
					(r % 2 == 0) ? n -= (depthCamWidth - 1) : n -= (depthCamWidth + 1);
				}
			}
		}
	}
	/*for (int y = 0; y < depthCamHeight - 1; y++) {
		if (y > 0) {
			// Degenerate begin: repeat first vertex
			//pVertexMem->indices.push_back((unsigned int)(y * depthCamHeight));
			indices.push_back((unsigned int)(y * depthCamHeight));
		}

		for (int x = 0; x < depthCamWidth; x++) {
			// One part of the strip
			//pVertexMem->indices.push_back((unsigned int)((y * depthCamHeight) + x));
			//pVertexMem->indices.push_back((unsigned int)(((y + 1) * depthCamHeight) + x));
			indices.push_back((unsigned int)((y * depthCamHeight) + x));
			indices.push_back((unsigned int)(((y + 1) * depthCamHeight) + x));
		}

		if (y < depthCamHeight - 2) {
			// Degenerate end: repeat last vertex
			indices.push_back((unsigned int)(((y + 1) * depthCamHeight) + (depthCamWidth - 1)));
		}
	}*/
}

void RawDepthPipeline::addIndexDataTriangleStrip()
{
	int lastIndex = 0;
	int direction = -1;
	int width = depthCamWidth * 2;
	int height = depthCamHeight - 1;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (x == 0)
			{
				// always place the first element of the row at the 
				// current index value
				indices.push_back((unsigned int)lastIndex);
			}
			else if (x % 2 == 0)
			{
				// subtract the number of indices
				// and the direction (+1 or -1)
				lastIndex -= width + direction;
				indices.push_back((unsigned int)lastIndex);
			}
			else if (x % 2 != 0)
			{
				// add the number of vertices in the row
				// to the index
				lastIndex += width;
				indices.push_back((unsigned int)lastIndex);
			}

			if (x == (width - 1) * y)
			{
				// add degenerate triangle (to help switch direction)
				indices.push_back((unsigned int)lastIndex);
				// we are changing the row and direction
				direction *= -1;
			}
		}
	}
		

}

pcl::PointCloud<pcl::PointXYZ>::Ptr RawDepthPipeline::createPointCloudPCL(PXCImage::ImageData ddepth)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ>(depthCamWidth, depthCamHeight));
	short * vertices = (short*)ddepth.planes[0];
	short x, y, z;
	for (int i = 0; i < depthCamWidth; i++)
	{
		for (int j = 0; j < depthCamHeight; j++)
		{
			
			unsigned int idx = 3 * (j*depthCamWidth + i);  // three coordinates per point
			x = vertices[idx];
			y = vertices[idx+1];
			z = vertices[idx+2];
			
			if (z && z < 2000)
			{

				// normalize vertices to meters
				cloud->points[idx / 3].x = ((float)x) / 1000.0;
				cloud->points[idx / 3].y = ((float)y) / 1000.0;
				cloud->points[idx / 3].z = ((float)z) / 1000.0;
			}
			
		}
	}

	return cloud;
}