#version 330

uniform mat4 TexGenMat;
uniform mat4 InvViewMat;

uniform mat4 P;
uniform mat4 MV;
uniform mat4 N;

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec2 inCoord;
layout (location = 2) in vec3 inNormal;

out vec3 vNormal, eyeVec;
out vec2 texCoord;
out vec4 projCoords;

void main()
{
	vNormal = (N * vec4(inNormal, 0.0)).xyz;

	vec4 posEye    = MV * vec4(inPosition, 1.0);
	vec4 posWorld  = InvViewMat * posEye;
	projCoords     = TexGenMat * posWorld;
	//projCoords     = TexGenMat * vec4(inPosition, 1.0);
	//projCoords     = TexGenMat * posEye;

	//projCoords     = TexGenMat * MV * vec4(inPosition, 1.0);
	// only needed for specular component
	// currently not used
	eyeVec = -posEye.xyz;

	gl_Position = P * MV * vec4(inPosition, 1.0);

	// normal texture
	texCoord = inCoord;
}