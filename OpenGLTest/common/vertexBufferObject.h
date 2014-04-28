#pragma once

#include <GL/glew.h>
#include <vector>

using namespace std;

class VertexBufferObject
{
public:
	VertexBufferObject();
	~VertexBufferObject();

	void createVBO(int a_iSize = 0);
	void releaseVBO();

	void* mapBufferToMemory(int iUsageHint);
	void* mapSubBufferToMemory(int iUsageHint, GLuint uiOffset, GLuint uiLength);
	void unmapBuffer();

	void bindVBO(int a_iBufferType = GL_ARRAY_BUFFER);
	void uploadDataToGPU(int iUsageHint);
	void updateGPUData(int startIndex);

	void addData(void* ptrData, GLuint uiDataSize);

	void* getDataPointer();
	GLuint getBufferID();

	int getCurrentSize();
private:
	GLuint uiBuffer;
	int iSize;
	int iCurrentSize;
	int iBufferType;
	vector<GLbyte> data;

	bool bDataUploaded;
};

