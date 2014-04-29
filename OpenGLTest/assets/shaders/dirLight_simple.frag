#version 330

smooth in vec3 vNormal;
out vec4 outputColor;

uniform sampler2D gSampler;
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
	


	float fDiffuseIntensity = max(0.0, dot(normalize(vNormal), -sunLight.vDirection));
	float fMult = clamp(sunLight.fAmbientIntensity + fDiffuseIntensity, 0.0, 1.0);
	if(sunLight.iSkybox == 1)
		fMult = sunLight.fAmbientIntensity;

	outputColor = vColor*fMult;
}