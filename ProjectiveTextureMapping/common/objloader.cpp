#include <fstream>
#include <vector>
#include <string>
#include <iostream>

// math lib
#include <glm/glm.hpp>

#include "objloader.h"

#include <assimp/Importer.hpp> // C++ importer interface
#include <assimp/scene.h> // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#include <GL/glew.h>

using namespace std;

/*bool bLoaded;
static CVertexBufferObject vboModelData;
static UINT uiVAO;
static vector<CTexture> tTextures;
vector<int> iMeshStartIndices;
vector<int> iMeshSizes;
vector<int> iMaterialIndices;
int iNumMaterials;
*/

bool importAsset(const string& pFile)
{
    // create an instance of the Importer class
    Assimp::Importer importer;

    // read the given file
    // with some postprocessing
    const aiScene* scene = importer.ReadFile(pFile,
          aiProcess_CalcTangentSpace    |
          aiProcess_Triangulate         |
          aiProcess_JoinIdenticalVertices |
          aiProcess_SortByPType);

    // check if the import failed
    if (!scene)
    {
        cout << "Importer Error: " << importer.GetErrorString() << endl;
        return false;
    }

    // access the file's contents

    return true;
}

bool loadOBJ(
    const char * path,
    vector < glm::vec3 > & out_vertices,
    vector < glm::vec2 > & out_uvs,
    vector < glm::vec3 > & out_normals
)
{
    vector<unsigned int> vertexIndices, uvIndices, normalIndices;
    vector< glm::vec3> temp_vertices;
    vector< glm::vec2> temp_uvs;
    vector< glm::vec3> temp_normals;
    
    // open the file
    FILE* file = fopen(path, "r");
    if (file == NULL)
    {
        printf("Impossible to open the file !\n");
        return false;
    }
    
    while (1)
    {
        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if(res == EOF)
            break;
        // else : parse lineheader
        if(strcmp(lineHeader, "v") == 0)
        {
            glm::vec3 vertex;
            fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, & vertex.z);
            temp_vertices.push_back(vertex);
        }
        else if(strcmp(lineHeader, "vt") == 0) 
        {
            glm::vec2 uv;
            fscanf(file, "%f %f\n", &uv.x, &uv.y);
            temp_uvs.push_back(uv);
        }
        else if(strcmp(lineHeader, "vn") == 0)
        {
            glm::vec3 normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
            temp_normals.push_back(normal);
        }
        else if(strcmp(lineHeader, "f") == 0)
        {
            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], 
                        &uvIndex[0], &normalIndex[0], &vertexIndex[1],
                        &uvIndex[1], &normalIndex[1], &vertexIndex[2],
                        &uvIndex[2], &normalIndex[2]);
            if (matches != 9)
            {
                printf("Matches: %d\n", matches);
                printf("File can't be read by the parser." 
                       "Try exporting with other options!\n");
                return false;
            }
            
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
            uvIndices.push_back(uvIndex[0]);
            uvIndices.push_back(uvIndex[1]);
            uvIndices.push_back(uvIndex[2]);
            normalIndices.push_back(normalIndex[0]);
            normalIndices.push_back(normalIndex[1]);
            normalIndices.push_back(normalIndex[2]);
        }
        
        // for each vertex of each triangle
        for(unsigned int i = 0; i < vertexIndices.size(); i++)
        {
            unsigned int vertexIndex = vertexIndices[i];
            
            // obj indexing starts at 1
            glm::vec3 vertex = temp_vertices[vertexIndex-1];
            
            out_vertices.push_back(vertex);
        }
        
        // for each uv of each triangle
        for(unsigned int i = 0; i < uvIndices.size(); i++)
        {
            unsigned int uvIndex = uvIndices[i];
            
            // obj indexing starts at 1
            glm::vec2 uv = temp_uvs[uvIndex-1];
            
            out_uvs.push_back(uv);
        }
        
        // for each normal of each triangle
        for(unsigned int i = 0; i < normalIndices.size(); i++)
        {
            unsigned int normalIndex = normalIndices[i];
            
            // obj indexing starts at 1
            glm::vec3 normal = temp_normals[normalIndex-1];
            
            out_normals.push_back(normal);
        }
    }
    
    return true;
}
