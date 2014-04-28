#pragma once

// OGL via glew
#include <GL/glew.h>
// include GLFW as the windowing context handling library
#include <GLFW/glfw3.h>
// include GLM math library
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class MarchingCubes
{
public:
	MarchingCubes();
	~MarchingCubes();

	struct GridCell
	{
		float val[8];
		glm::vec3 pos[8];
	};

	void RenderMarchingCube(float *data, glm::ivec3 size, glm::ivec3 gridSize, float isoLevel);
	int Polygonise(GridCell &grid, float isolevel, glm::vec3 *triangles);

	static const int edgeTable[256];
	static const int triTable[256][16];
private:
	glm::vec3 VertexInterp(float isolevel, glm::vec3 p1, glm::vec3 p2, float valp1, float valp2);
};

