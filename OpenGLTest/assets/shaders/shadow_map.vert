#version 330

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec2 inCoord;
layout (location = 2) in vec3 inNormal;

// output data: interpolated
out vec2 texCoord;
out vec4 shadowCoord;
smooth out vec3 vNormal;

// uniforms
uniform mat4 P;
uniform mat4 MV;
uniform mat4 N;
uniform mat4 depthBiasMVP;

void main()
{
	gl_Position = P * MV * vec4(inPosition, 1.0);

	shadowCoord = depthBiasMVP * vec4(inPosition, 1.0);

	// uv coords
	texCoord = inCoord;

	// calculate normals
	vec4 vRes = N*vec4(inNormal, 0.0);
	vNormal = vRes.xyz;
}