#include "vertexBufferObject.h"


VertexBufferObject::VertexBufferObject()
{
	bDataUploaded = false;
}


VertexBufferObject::~VertexBufferObject()
{
	releaseVBO();
}

void VertexBufferObject::createVBO(int a_iSize)
{
	glGenBuffers(1, &uiBuffer);
	data.reserve(a_iSize);
	iSize = a_iSize;
	iCurrentSize = 0;
}

void VertexBufferObject::releaseVBO()
{
	glDeleteBuffers(1, &uiBuffer);
	bDataUploaded = false;
	data.clear();
}

void* VertexBufferObject::mapBufferToMemory(int iUsageHint)
{
	if (!bDataUploaded) return NULL;
	void* ptrRes = glMapBuffer(iBufferType, iUsageHint);
	return ptrRes;
}

void* VertexBufferObject::mapSubBufferToMemory(int iUsageHint, GLuint uiOffset, GLuint uiLength)
{
	if (!bDataUploaded) return NULL;
	void* ptrRes = glMapBufferRange(iBufferType, uiOffset, uiLength, iUsageHint);
	return ptrRes;
}

void VertexBufferObject::unmapBuffer()
{
	glUnmapBuffer(iBufferType);
}

void VertexBufferObject::bindVBO(int a_iBufferType)
{
	iBufferType = a_iBufferType;
	glBindBuffer(iBufferType, uiBuffer);
}

void VertexBufferObject::uploadDataToGPU(int iDrawingHint)
{
	glBufferData(iBufferType, data.size(), &data[0], iDrawingHint);
	bDataUploaded = true;
	data.clear();
}

void VertexBufferObject::updateGPUData(int startIndex)
{
	glBufferSubData(iBufferType, startIndex, data.size(), &data[0]);
	bDataUploaded = true;
	data.clear();
}

void VertexBufferObject::addData(void* ptrData, GLuint uiDataSize)
{
	data.insert(data.end(), (GLbyte*)ptrData, (GLbyte*)ptrData + uiDataSize);
	iCurrentSize += uiDataSize;
}

void* VertexBufferObject::getDataPointer()
{
	if (bDataUploaded) return NULL;
	return (void*)data[0];
}

GLuint VertexBufferObject::getBufferID()
{
	return uiBuffer;
}

int VertexBufferObject::getCurrentSize()
{
	return iCurrentSize;
}