#pragma once

#include <assimp/Importer.hpp>  // C++ importer interface
#include <assimp/scene.h>		// output data structure
#include <assimp/postprocess.h> // post process fla

#include <GL/glew.h>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>



#include "vertexBufferObject.h"
#include "texture.h"

using namespace std;

class AssimpModel
{
public:
	AssimpModel();
	~AssimpModel();

	bool LoadModelFromFile(char* sFilePath);
	bool LoadModelFromFileBumpMap(char* sFilePath, char* sColorMap, char* sNormalMap);

	static void FinalizeVBO();
	static void BindModelsVAO();

	void RenderModel();
	void RenderModel(int iModelViewLoc, int iNormalMatrixLoc, glm::mat4 mCurrent, glm::mat4 mModelView);
	void RenderModelBumpMap(GLuint shaderProgramID);
private:
	bool bLoaded;
	static VertexBufferObject vboModelData;
	static GLuint uiVAO;
	static vector<Texture> tTextures;
	vector<int> iMeshStartIndices;
	vector<int> iMeshSizes;
	vector<int> iMaterialIndices;
	int iNumMaterials;
};

