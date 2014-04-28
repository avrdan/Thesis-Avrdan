#version 330

layout (points) in;
layout (triangle_strip, max_vertices = 16) out;

//in vec4  positionPass[];
//out vec4 position;
//varying vec4 position;

// uniforms

// volume data field texture
uniform sampler3D  dataFieldTex;
// edge table texture
uniform isampler2D edgeTableTex;
// triangles table texture
uniform isampler2D triTableTex;
// global iso level
uniform float isoLevel;
// marching cubes vertices decal
uniform vec3 vertDecals[8];

uniform struct Matrices
{
	mat4 projMatrix;
	mat4 modelMatrix;
	mat4 viewMatrix;
} matrices;

// get vertex i position within
// current marching cube
vec3 cubePos(int i)
{
	return gl_in[0].gl_Position.xyz + vertDecals[i];
}

// get vertex i value within
// current marching cube
float cubeVal(int i)
{
	// compute scalar field values, per vertex
	return texture(dataFieldTex, (cubePos(i) + 1.0) / 2.0).r;
}

// get edge table value
/*int edgeTableValue(int i)
{
	return texelFetch(edgeTableTex, ivec2(i, 0), 0).r;
}*/

// get triangle table value
int triTableValue(int i, int j)
{
	return texelFetch(triTableTex, ivec2(j, i), 0).r;
}

// compute interpolated vertex along an edge
vec3 vertexInterp(float isoLevel, vec3 v0, float l0, vec3 v1, float l1)
{
	return mix(v0, v1, (isoLevel - l0) / (l1 - l0));
}

void main()
{
	int cubeIndex = 0;

	float cubeVal0 = cubeVal(0);
	float cubeVal1 = cubeVal(1);
	float cubeVal2 = cubeVal(2);
	float cubeVal3 = cubeVal(3);
	float cubeVal4 = cubeVal(4);
	float cubeVal5 = cubeVal(5);
	float cubeVal6 = cubeVal(6);
	float cubeVal7 = cubeVal(7);

	// determine the index into the
	// edge table which tells us which
	// vertices are inside of the surface
	cubeIndex += int(cubeVal0 < isoLevel);
	cubeIndex += int(cubeVal1 < isoLevel)*2;
	cubeIndex += int(cubeVal2 < isoLevel)*4;
	cubeIndex += int(cubeVal3 < isoLevel)*8;
	cubeIndex += int(cubeVal4 < isoLevel)*16;
	cubeIndex += int(cubeVal5 < isoLevel)*32;
	cubeIndex += int(cubeVal6 < isoLevel)*64;
	cubeIndex += int(cubeVal7 < isoLevel)*128;

	// cube is entirely in/out of the surface
	if (cubeIndex == 0 || cubeIndex == 255)
		return;

	vec3 vertList[12];

	//Find the vertices where the surface intersects the cube
	vertList[0]  = vertexInterp(isoLevel, cubePos(0), cubeVal0, cubePos(1), cubeVal1);
	vertList[1]  = vertexInterp(isoLevel, cubePos(1), cubeVal1, cubePos(2), cubeVal2);
	vertList[2]  = vertexInterp(isoLevel, cubePos(2), cubeVal2, cubePos(3), cubeVal3);
	vertList[3]  = vertexInterp(isoLevel, cubePos(3), cubeVal3, cubePos(0), cubeVal0);
	vertList[4]  = vertexInterp(isoLevel, cubePos(4), cubeVal4, cubePos(5), cubeVal5);
	vertList[5]  = vertexInterp(isoLevel, cubePos(5), cubeVal5, cubePos(6), cubeVal6);
	vertList[6]  = vertexInterp(isoLevel, cubePos(6), cubeVal6, cubePos(7), cubeVal7);
	vertList[7]  = vertexInterp(isoLevel, cubePos(7), cubeVal7, cubePos(4), cubeVal4);
	vertList[8]  = vertexInterp(isoLevel, cubePos(0), cubeVal0, cubePos(4), cubeVal4);
	vertList[9]  = vertexInterp(isoLevel, cubePos(1), cubeVal1, cubePos(5), cubeVal5);
	vertList[10] = vertexInterp(isoLevel, cubePos(2), cubeVal2, cubePos(6), cubeVal6);
	vertList[11] = vertexInterp(isoLevel, cubePos(3), cubeVal3, cubePos(7), cubeVal7);

	mat4 mMVP = matrices.projMatrix * matrices.viewMatrix * matrices.modelMatrix;

	int i = 0;
	vec4 position;
	// create the triangle
	//while(true){
	//for(i=0; i<16; i+=3){
	//	if(triTableValue(cubeIndex, i) != -1)
	for(i = 0; triTableValue(cubeIndex, i) != -1; i += 3)
	{
			position    = vec4(vertList[triTableValue(cubeIndex, i)], 1);
			//position    = vec4(vertList[triTableValue(cubeIndex, i%3)], 1);
			gl_Position = mMVP * position;
			// generate first vertex
			EmitVertex();

			//generate second vertex of triangle
			position = vec4(vertList[triTableValue(cubeIndex, (i+1))], 1);
			//position    = vec4(vertList[triTableValue(cubeIndex, i%3)], 1);
			gl_Position = mMVP * position;
			EmitVertex();

			// generate last vertex
			position = vec4(vertList[triTableValue(cubeIndex, (i+2))], 1);
			//position    = vec4(vertList[triTableValue(cubeIndex, i%3)], 1);
			gl_Position = mMVP * position;
			EmitVertex();

			// end triangle strip
			EndPrimitive();
		/*} 
		else {
			//continue;
			break;
		}*/

		//i += 3;
	}
}