#version 330

in vec2 texCoord;
in vec4 shadowCoord;
smooth in vec3 vNormal;

// color
out vec4 outputColor;

uniform sampler2D gSampler;
uniform sampler2DShadow shadowMap;
uniform vec4 vColor;

struct DirectionalLight
{
	vec3 vColor;
	vec3 vDirection;
	float fAmbientIntensity;
	int iSkybox;
};

uniform DirectionalLight sunLight;

void main()
{
	vec4 vTexColor = texture(gSampler, texCoord);
	//vec4 vTexColor = vec4(texCoord.x, texCoord.y, 0, 1.0);
	float fDiffuseIntensity = max(0.0, dot(normalize(vNormal), -sunLight.vDirection));
	float fMult = clamp(sunLight.fAmbientIntensity + fDiffuseIntensity, 0.0, 1.0);
	if(sunLight.iSkybox == 1)
		fMult = sunLight.fAmbientIntensity;

	float visibility = texture(shadowMap, vec3(shadowCoord.st, shadowCoord.r / shadowCoord.q) );

	/*if (visibility == 1.0)
	{
		outputColor = vec4(1, 0, 0, 1);
	}
	else
	{
	outputColor = vec4(0, 1, 0, 1);
	}*/

	//vec4 x = texture(shadowMap, vec3(shadowCoord.st, shadowCoord.r / shadowCoord.q) );
	//outputColor = x;
	
	//outputColor = vec4(shadowCoord.s / shadowCoord.q, shadowCoord.t / shadowCoord.q, shadowCoord.r, 1.0);
	//outputColor = texture(gSampler, vec4(shadowCoord.s / shadowCoord.q, shadowCoord.t / shadowCoord.q, shadowCoord.r, 1.0));
	outputColor = visibility*vTexColor*vColor*fMult;
	//outputColor = vTexColor*vColor*fMult;
	//outputColor = vec4(visibility, visibility, visibility, 1.0)*vColor*fMult;
}