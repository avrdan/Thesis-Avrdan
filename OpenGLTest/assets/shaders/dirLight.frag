#version 330

in vec2 texCoord;
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
	
	vec4 vTexColor = texture(gSampler, texCoord);
	//vec4 vTexColor = vec4(texCoord.x, texCoord.y, 0, 1.0);
	float fDiffuseIntensity = max(0.0, dot(normalize(vNormal), -sunLight.vDirection));
	float fMult = clamp(sunLight.fAmbientIntensity + fDiffuseIntensity, 0.0, 1.0);
	if(sunLight.iSkybox == 1)
		fMult = sunLight.fAmbientIntensity;
	//outputColor = vTexColor*vColor*vec4(sunLight.vColor * (sunLight.fAmbientIntensity + fDiffuseIntensity), 1.0);
	outputColor = vTexColor*vColor*fMult;
}