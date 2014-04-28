#include "assimpModel.h"
#include <iostream>

VertexBufferObject AssimpModel::vboModelData;
GLuint AssimpModel::uiVAO;
vector<Texture> AssimpModel::tTextures;

string GetDirectoryPath(string sFilePath)
{
	// Get directory path
	string sDirectory = "";
	
	for (unsigned int i = sFilePath.size() - 1; i >=0 ; i--)
	{
		if (sFilePath[i] == '\\' || sFilePath[i] == '/')
		{
			sDirectory = sFilePath.substr(0, i + 1);
			break;
		}
	}
	
	return sDirectory;
}

AssimpModel::AssimpModel()
{
	bLoaded = false;
}


AssimpModel::~AssimpModel()
{
}

bool AssimpModel::LoadModelFromFile(char* sFilePath)
{
	if (vboModelData.getBufferID() == 0)
	{
		vboModelData.createVBO();
		tTextures.reserve(50);
	}

	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(sFilePath,
		aiProcess_CalcTangentSpace |
		aiProcess_Triangulate |
		aiProcess_JoinIdenticalVertices |
		aiProcess_SortByPType);

	if (!scene)
	{
		cout << "Couldn't load model. Error importing asset: " << sFilePath << endl;
		return false;
	}

	const int iVertexTotalSize = sizeof(aiVector3D)* 2 + sizeof(aiVector2D);
	int iTotalVertices = 0;

	for (unsigned int i = 0; i < scene->mNumMeshes; i++)
	{
		aiMesh* mesh = scene->mMeshes[i];
		int iMeshFaces = mesh->mNumFaces;
		iMaterialIndices.push_back(mesh->mMaterialIndex);
		int iSizeBefore = vboModelData.getCurrentSize();
		iMeshStartIndices.push_back(iSizeBefore / iVertexTotalSize);

		for (int j = 0; j < iMeshFaces; j++)
		{
			const aiFace& face = mesh->mFaces[j];
			for (int k = 0; k < 3; k++)
			{
				aiVector3D pos = mesh->mVertices[face.mIndices[k]];
				aiVector3D uv = mesh->mTextureCoords[0][face.mIndices[k]];
				aiVector3D normal = mesh->HasNormals() ? mesh->mNormals[face.mIndices[k]] : aiVector3D(1.0f, 1.0f, 1.0f);
				vboModelData.addData(&pos, sizeof(aiVector3D));
				vboModelData.addData(&uv, sizeof(aiVector2D));
				vboModelData.addData(&normal, sizeof(aiVector3D));
			}
		}

		int iMeshVertices = mesh->mNumVertices;
		iTotalVertices += iMeshVertices;
		iMeshSizes.push_back((vboModelData.getCurrentSize() - iSizeBefore) / iVertexTotalSize);
	}

	iNumMaterials = scene->mNumMaterials;
	vector<int> materialRemap(iNumMaterials);

	for (int i = 0; i < iNumMaterials; i++)
	{
		const aiMaterial* material = scene->mMaterials[i];
		int a = 5;
		int texIndex = 0;
		aiString path;	// filename

		if (material->GetTexture(aiTextureType_DIFFUSE, texIndex, &path) == AI_SUCCESS)
		{
			string sDir = GetDirectoryPath(sFilePath);
			string sTextureName = path.data;
			string sFullPath = sDir + sTextureName;
			//string sFullPath = sTextureName;
			cout << "FULL TEXTURE PATH: " << sTextureName << endl;
			int iTexFound = -1;
			for (unsigned int j = 0; j < tTextures.size(); j++)
			{
				if (sFullPath == tTextures[j].getPath())
				{
					iTexFound = j;
					break;
				}
			}
			if (iTexFound != -1)
				materialRemap[i] = iTexFound;
			else
			{
				Texture tNew;
				tNew.loadTexture2D(sFullPath, true);
				materialRemap[i] = tTextures.size();
				tTextures.push_back(tNew);
			}
		}
	}

	for (unsigned int i = 0; i < iMeshSizes.size(); i++)
	{
		int iOldIndex = iMaterialIndices[i];
		iMaterialIndices[i] = materialRemap[iOldIndex];
	}

	return bLoaded = true;
}

void AssimpModel::FinalizeVBO()
{
	glGenVertexArrays(1, &uiVAO);
	glBindVertexArray(uiVAO);
	vboModelData.bindVBO();
	vboModelData.uploadDataToGPU(GL_STATIC_DRAW);

	// vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(aiVector3D)+sizeof(aiVector2D), 0);
	// texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(aiVector3D)+sizeof(aiVector2D), (void*)sizeof(glm::vec3));
	// normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(aiVector3D)+sizeof(aiVector2D), (void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));
}

void AssimpModel::RenderModel()
{
	if (!bLoaded)
		return;

	int iNumMeshes = iMeshSizes.size();

	for (int i = 0; i < iNumMeshes; i++)
	{
		int iMatIndex = iMaterialIndices[i];
		tTextures[iMatIndex].bindTexture();
		tTextures[iMatIndex].setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_BILINEAR_MIPMAP);
		//cout << "Mesh start: " << iMeshStartIndices[i] << endl;
		//cout << "Mesh size:" << iMeshSizes[i] << endl;

		glDrawArrays(GL_TRIANGLES, iMeshStartIndices[i], iMeshSizes[i]);
	}
}

void AssimpModel::RenderModel(int iModelViewLoc, int iNormalMatrixLoc, glm::mat4 mCurrent, glm::mat4 mModelView)
{
	if (!bLoaded)
		return;

	int iNumMeshes = iMeshSizes.size();

	for (int i = 0; i < iNumMeshes; i++)
	{
		int iMatIndex = iMaterialIndices[i];
		tTextures[iMatIndex].bindTexture();

		glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
		glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*mCurrent));

		glDrawArrays(GL_TRIANGLES, iMeshStartIndices[i], iMeshSizes[i]);
	}
}

void AssimpModel::BindModelsVAO()
{
	glBindVertexArray(uiVAO);
}