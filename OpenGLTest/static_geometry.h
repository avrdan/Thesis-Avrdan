#ifndef STATIC_GEOMETRY_H
#define STATIC_GEOMETRY_H

#include <glm/glm.hpp>
#include "common/vertexBufferObject.h"
#include <GL/glew.h>

int GenerateTorus(VertexBufferObject &vboDest, float fRadius, float fTubeRadius, int iSubDivAround, int iSubDivTube);
int GenerateCylinder(VertexBufferObject &vboDest, float fRadius, float fHeight, int iSubDivAround, float fMapU = 1.0f, float fMapV = 1.0f);
void CreateStaticSceneObjects(GLuint* uiVAO, VertexBufferObject& vboDest);
void CreateStaticSceneObjects2(GLuint* uiVAO, VertexBufferObject& vboDest, glm::vec3 frustum_coords[]);
void CreateStaticSceneObjects3(GLuint* uiVAO, VertexBufferObject& vboDest);
void CreateFrustumOutlines(GLuint* uiVAO, VertexBufferObject& vboDest, glm::vec3 frustum_coords[]);
void CreateFrustumSimple(GLuint* uiVAO, VertexBufferObject& vboDest, glm::vec3 frustum_coords[]);

extern int iSphereFaces;

glm::vec3 vCubeVertices[];
glm::vec2 vCubeTexCoords[];
glm::vec3 vPyramidVertices[];
glm::vec2 vPyramidTexCoords[];
glm::vec3 vGround[];

// lighting scene
glm::vec3 vCubeVerticesL[];
glm::vec2 vCubeTexCoordsL[];
glm::vec3 vCubeNormalsL[];
glm::vec3 vGroundL[];

// asset loader
glm::vec2 vCubeTexCoordsAS[];

glm::vec3 ss_quad_pos[];
glm::vec2  ss_quad_st[];

glm::vec3 g_quad_vertex_buffer_data[];

glm::vec3 vBuildingNormals[];
glm::vec3 vBuilding[];


#endif