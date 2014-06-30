#version 330

uniform mat4 TexGenMat;
uniform mat4 InvViewMat;

uniform mat4 P;
uniform mat4 MV;
uniform mat4 N;

uniform mat4 kRGB;
uniform mat4 kDepthInv;
uniform mat4 depthToColor;

layout (location = 0) in vec3 inPosition;

out vec2 texCoord;
out vec4 projCoords;

void main()
{
	vec4 posEye    = MV * vec4(inPosition, 1.0);
	vec4 posWorld  = InvViewMat * posEye;
	projCoords     = TexGenMat * posWorld;

	//gl_Position = P * MV * depthToColor * vec4(inPosition, 1.0);
	//gl_Position = P * MV * depthToColor * kDepthInv * vec4(inPosition, 1.0);
	//gl_Position = P * MV * vec4(inPosition, 1.0);
	//gl_Position   = P * MV * kRGB * kDepthInv * vec4(inPosition, 1.0); 
	//gl_Position = P * MV * depthToColor * kRGB * kDepthInv * vec4(inPosition, 1.0);
	//gl_Position = P * MV * depthToColor * vec4(inPosition, 1.0);
	gl_Position = P * MV * vec4(inPosition, 1.0);
	 
}