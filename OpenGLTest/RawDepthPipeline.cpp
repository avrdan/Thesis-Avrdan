#include "RawDepthPipeline.h"


RawDepthPipeline::RawDepthPipeline()
{
	// init data structure
	pVertexMem = new sVertexType[depthCamWidth * depthCamHeight];

	//pp.EnableImage(PXCImage::COLOR_FORMAT_VERTICES);
	pp.EnableImage(PXCImage::COLOR_FORMAT_DEPTH);
	
	// also enable color image
	//pp.EnableImage(PXCImage::COLOR_FORMAT_RGB32);

	pp.Init();
	//UtilRender depth_render(L"Depth Stream");
	//depth_render = UtilRender(L"Depth Stream");
	depth_render = new UtilRender(L"Depth Stream");
	//color_render = new UtilRender(L"Color Stream");
	
	pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_SMOOTHING, true);
	pp.QueryCapture()->QueryDevice()->QueryProperty(PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE, &dvalues[0]);
	pp.QueryCapture()->QueryDevice()->QueryProperty(PXCCapture::Device::PROPERTY_DEPTH_SATURATION_VALUE, &dvalues[1]);
	//pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_SENSOR_RANGE, new float[]{200, 1000});
	//pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE, 500);
	//pp.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE, 100);
	// set framerate to 60 fps
	pp.QueryCapture()->QueryVideoStream(0)->QueryProfile(&pinfo);
	pinfo.frameRateMin.numerator = 60;
	pinfo.frameRateMin.denominator = 1;
	pinfo.frameRateMax = pinfo.frameRateMin;
	pp.QueryCapture()->QueryVideoStream(0)->SetProfile(&pinfo);
	//pp.QueryCapture()->QueryVideoStream(1)->SetProfile(&pinfo);
	
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
	nPointsRGB = rgbCamWidth*rgbCamHeight;
	//pos3dDepth = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	pos2d	    = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	pos3d		= (PXCPoint3DF32*)new PXCPoint3DF32[nPointsRGB];
	pos2dColor  = (PXCPointF32*)new PXCPointF32[nPoints];

	depthData.resize(nPointsRGB);
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

	addIndexData(depthCamWidth, depthCamHeight);
	//addIndexData(rgbCamWidth, rgbCamHeight);
	//addIndexData();
	//addIndexDataTriangles();
	//addIndexDataTriangleStrip();
	nSize = 5;
	averageFrameCount = 20;
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

void RawDepthPipeline::renderFrame(bool useMedianFiltering, bool useWeightedMovingAverage)
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
	//if (!color_render->RenderFrame(color_image)) return;
	if (!depth_render->RenderFrame(depth_image)) return;


	// get depth data
	PXCImage::ImageData ddepth;
	PXCImage::ImageData dcolor;

	depth_image->AcquireAccess(PXCImage::ACCESS_READ, &ddepth);
	//color_image->AcquireAccess(PXCImage::ACCESS_READ, &dcolor);
	//createPointCloud(ddepth);
	createPointCloudMappedToWorld(ddepth, useMedianFiltering, useWeightedMovingAverage);
	//createPointCloudMappedToWorld(ddepth, dcolor);
	// NOTE THAT NOT ALL ARRAY VALUES ARE VALID FOR
	// JUST THE ONES WITH DEPTH > 10
	//printPointCloudData();
	//printSelectivePointCloudData(100);

	depth_image->ReleaseAccess(&ddepth);
	//color_image->ReleaseAccess(&dcolor);
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

void RawDepthPipeline::createPointCloudMappedToWorld(PXCImage::ImageData ddepth, bool useMedianFiltering, bool useWeightedMovingAverage)
{
	pos2d = (PXCPoint3DF32*)new PXCPoint3DF32[depthCamWidth*depthCamHeight];
	int n = 0;

	// find depth image stride
	int depthStride = ddepth.pitches[0] / sizeof(pxcU16);
	pxcU16 lastDepthValue = 0;

	for (int y = 0; y < depthCamHeight; y++)
	{
		for (int x = 0; x < depthCamWidth; x++)
		{
			pxcU16 depthValue = ((pxcU16*)ddepth.planes[0])[y * depthStride + x];

			// raw depth data
			
			if (x == depthCamWidth / 2 && y == depthCamHeight / 2)
				centerDepth = depthValue;

			lastDepthValue = depthValue;
			if (depthValue > 10 && depthValue < 2000)
			{
				pos2d[n].x = (pxcF32)x;
				pos2d[n].y = (pxcF32)y;
				pos2d[n].z = (pxcF32)depthValue;	
			}
			n++;
		}
	}

	n = 0;

	// apply median filter (requires another pass)
	// spatial smoothing
	if (useMedianFiltering)
		pos2d = medianFilter();

	// use weighted moving average
	// temporal smoothing
	if (useWeightedMovingAverage)
		pos2d = weightedMovingAverage();

	projection->ProjectImageToRealWorld(nPoints, &pos2d[0], worldPos.data());
	
	// cleanup
	delete[] pos2d;
}

void RawDepthPipeline::createPointCloudMappedToWorld(PXCImage::ImageData ddepth, PXCImage::ImageData dcolor)
{
	worldPos.clear();
	depthData.clear();
	depthData.resize(nPointsRGB);
	//pos2d = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	pos2d		= (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	pos2dColor  = (PXCPointF32*)new PXCPointF32[nPoints];
	pos3d		= (PXCPoint3DF32*)new PXCPoint3DF32[nPointsRGB];

	int n = 0;
	// find depth image stride
	int depthStride = ddepth.pitches[0] / sizeof(pxcU16);
	pxcU16 lastDepthValue = 0;

	for (int y = 0; y < depthCamHeight; y++)
	{
		for (int x = 0; x < depthCamWidth; x++)
		{
			pxcU16 depthValue = ((pxcU16*)ddepth.planes[0])[y * depthStride + x];
			// raw depth data

			lastDepthValue = depthValue;
			if (depthValue > 10 && depthValue < 2000)
			{
				pos2d[n].x = (pxcF32)x;
				pos2d[n].y = (pxcF32)y;
				pos2d[n].z = (pxcF32)depthValue;
			}
			n++;
		}
	}

	// convert to color first
	projection->MapDepthToColorCoordinates(nPoints, &pos2d[0], &pos2dColor[0]);

	// save the aligned depth map
	n = 0;
	for(int y = 0; y < depthCamHeight; y++)
	{
		for (int x = 0; x < depthCamWidth; x++)
		{
			int xx = (int)(pos2dColor[n].x + 0.5f);
			int yy = (int)(pos2dColor[n].y + 0.5f);
			int currIndex = yy * depthCamWidth + xx;
			//depthData.at(currIndex) = 0;
			
			if (xx < 0 || yy < 0 || (pxcU32)xx >= rgbCamWidth || (pxcU32)yy >= rgbCamHeight)
				continue; // no mapping based on clipping due to differences in FOV between the two cameras.
			if (pos2d[n].z == dvalues[0] || pos2d[n].z == dvalues[1])
				continue; // no mapping based on unreliable depth values

			// save the mapped depth data
			if (pos2d[n].z > 0)
				depthData.at(currIndex) = pos2d[n].z;

			n++;
		}
	}


	//pos3d = &pos2dColor[0];
	n = 0;
	/*for (int y = 0; n < depthCamHeight*depthCamWidth; n++)
	{
		pos3d[n].x = pos2dColor[n].x;
		pos3d[n].y = pos2dColor[n].y;
		pos3d[n].z = pos2d[n].z;
	}*/
	for (int y = 0; y < rgbCamHeight; y++)
	{
		for (int x = 0; x < rgbCamWidth; x++)
		{
			if (depthData[n] > 0)
			{
				pos3d[n].x = x;
				pos3d[n].y = y;
				pos3d[n].z = depthData[n];
			}
			
			n++;
		}
	}
	//projection->ProjectImageToRealWorld(nPoints, screenPos.data(), worldPos.data());
	
	projection->ProjectImageToRealWorld(nPoints, &pos3d[0], worldPos.data());
	//projection->ProjectImageToRealWorld(nPoints, &pos2d[0], worldPos.data());

	// use this to align depth with color
	//projection->MapDepthToColorCoordinates

	delete[] pos2d;
	delete[] pos2dColor;
	delete[] pos3d;

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

			if (depthValue && depthValue < 1500 && (depthValue != dvalues[0] && depthValue != dvalues[1]))
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

//void RawDepthPipeline::addIndexData()
void RawDepthPipeline::addIndexData(int width, int height)
{
	int n = 0;
	int colSteps = width * 2;
	int rowSteps = height - 1;
	for (int r = 0; r < rowSteps; r++) {
		for (int c = 0; c < colSteps; c++) {
			int t = c + r * colSteps;

			if (c == colSteps - 1) {
				indices.push_back(n);
			}
			else {
				indices.push_back(n);

				if (t % 2 == 0) {
					n += width;
				}
				else {
					(r % 2 == 0) ? n -= (width - 1) : n -= (width + 1);
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

//sort the window using insertion sort
//insertion sort is best for this sorting
void RawDepthPipeline::insertionSort(int window[])
{
	int temp, i, j;
	for (i = 0; i < windowSize; i++){
		temp = window[i];
		for (j = i - 1; j >= 0 && temp < window[j]; j--){
			window[j + 1] = window[j];
		}
		window[j + 1] = temp;
	}
}

PXCPoint3DF32*  RawDepthPipeline::medianFilter()
{
	newPos2d = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	bool skip = false;

	int size = sqrt(windowSize);
	int midIndex = (size - 1) / 2;

	/*Concurrency::parallel_for(midIndex, depthCamHeight - midIndex,
		[&](int y)
	{
		Concurrency::parallel_for(midIndex, depthCamWidth - midIndex,
			[&](int x)
		{
		*/	
	
	for (int y = midIndex; y < depthCamHeight - midIndex; y++)
	{
		for (int x = midIndex; x < depthCamWidth - midIndex; x++)
		{
			int index = 0;
	
			for (int i = 0; i < size; i++)
			{
				for (int j = 0; j < size; j++)
				{
					window[index] = pos2d[(y - midIndex + i) * depthCamWidth + (x - midIndex + j)].z;
					index++;
				}
			}

			/*Concurrency::parallel_for(0, size,
				[&](int i)
			{
				Concurrency::parallel_for(0, size,
					[&](int j)
				{
					window[index] = pos2d[(y - midIndex + i) * depthCamWidth + (x - midIndex + j)].z;
					index++;
				});
			});*/

			for (int i = 0; i < windowSize; i++)
			{
				if (window[i] < 10 || window[i] > 2000)
				{
					skip = true;
					break;
				}
					
			}

			if (skip)
			{
				skip = false;
				continue;
			}
			else
			{
				// sort the window to find median
				insertionSort(window);

				// assign the median to centered element of the matrix
				newPos2d[y * depthCamWidth + x].z = (pxcF32)window[(windowSize - 1) / 2];
				newPos2d[y * depthCamWidth + x].x = (pxcF32)x;
				newPos2d[y * depthCamWidth + x].y = (pxcF32)y;
			}


		}
	}
	//	});
	//});

	delete[] pos2d;
	return newPos2d;
	//return pos2d;
}

PXCPoint3DF32* RawDepthPipeline::weightedMovingAverage()
{
	// push array to the back of the list
	averageList.push_back(pos2d);
	
	// check if we have to remove the oldest depth array
	checkForDequeue();

	sumDepthArray = (pxcF32*)new pxcF32[nPoints];
	
	for (int i = 0; i < nPoints; i++)
	{
		sumDepthArray[i] = 0;
	}
	
	averagedDepthArray = (PXCPoint3DF32*)new PXCPoint3DF32[nPoints];
	
	// divide by this to average the weights
	int denominator = 0;
	// priority
	int count = 1;

	// We first create a single array, summing all of the pixels of each frame on a weighted basis
	// and determining the denominator that we will be using later.
	for each (PXCPoint3DF32* item in averageList)
	{

		// process each row in paralel
		Concurrency::parallel_for(0, depthCamHeight,
			[&](int depthArrayRowIndex)
		{
			// Process each pixel in the row
			for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < depthCamWidth; depthArrayColumnIndex++)
			{
				int index = depthArrayColumnIndex + (depthArrayRowIndex * depthCamWidth);

				if (sumDepthArray[index] == 0 && count > 1)
					continue;

				if (item[index].z > 100 && item[index].z < 2000)
				{
					sumDepthArray[index] += (pxcF32)item[index].z * count;
				}
				else
					sumDepthArray[index] = 0;
			}
		});

		denominator += count;
		count++;
	}

	// Once we have summed all of the information on a weighted basis, we can divide each pixel
	// by our calculated denominator to get a weighted average.
	// Process each row in parallel
	Concurrency::parallel_for(0, depthCamHeight,
		[&](int depthArrayRowIndex)
	{
		// Process each pixel in the row
		for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < depthCamWidth; depthArrayColumnIndex++)
		{
			int index = depthArrayColumnIndex + (depthArrayRowIndex * depthCamWidth);
			
			if (sumDepthArray[index] > 10*averageFrameCount && sumDepthArray[index] <= 5000000)
			{
				averagedDepthArray[index].x = depthArrayColumnIndex;
				averagedDepthArray[index].y = depthArrayRowIndex;
				//if (sumDepthArray[index] / denominator > 10)
					averagedDepthArray[index].z = (pxcF32)(sumDepthArray[index] / denominator);
			}
		}
	});

	delete[] sumDepthArray;
	return averagedDepthArray;
}

void RawDepthPipeline::checkForDequeue()
{
	// We will recursively check to make sure we have Dequeued enough frames.
	// This is due to the fact that a user could constantly be changing the UI element
	// that specifies how many frames to use for averaging.
	if (averageList.size() > averageFrameCount)
	{
		delete[] averageList.front();
		averageList.pop_front();
		checkForDequeue();
	}
}