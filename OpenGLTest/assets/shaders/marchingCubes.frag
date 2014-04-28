#version 330

//smooth in vec2 vTexCoord;
//smooth in vec3 vNormal;
//smooth in vec3 vEyeSpacePos;
//smooth in vec3 vWorldPos;

//in vec4 position;
//in vec4 position;

out vec4 outputColor;

//uniform sampler2D gSampler;
uniform sampler3D dataFieldTex;
uniform vec3 dataStep;

uniform vec4 vColor;

/*struct DirectionalLight
{
	vec3 vColor;
	vec3 vDirection;
	float fAmbientIntensity;
	int iSkybox;
};*/

//uniform DirectionalLight sunLight;

void main()
{
	/*vec3 grad = vec3(
		texture(dataFieldTex, (position.xyz+vec3(dataStep.x, 0, 0)+1.0)/2.0).r - texture(dataFieldTex, (position.xyz+vec3(-dataStep.x, 0, 0)+1.0)/2.0).r, 
		texture(dataFieldTex, (position.xyz+vec3(0, dataStep.y, 0)+1.0)/2.0).r - texture(dataFieldTex, (position.xyz+vec3(0, -dataStep.y, 0)+1.0)/2.0).r, 
		texture(dataFieldTex, (position.xyz+vec3(0,0,dataStep.z)+1.0)/2.0).r - texture(dataFieldTex, (position.xyz+vec3(0,0,-dataStep.z)+1.0)/2.0).r);
	
	vec3 vNormal = normalize(grad);
	vec3 color   = vColor.rgb * abs(vNormal)*0.5;
	*/

	/*vec3 vNormalized = normalize(vNormal);

	vec4 vTexColor = texture(gSampler, vTexCoord);
	vec4 vMixedColor = vTexColor * vColor;

	float fDiffuseIntensity = max(0.0, dot(vNormal, -sunLight.vDirection));
	float fMult = clamp(sunLight.fAmbientIntensity+fDiffuseIntensity, 0.0, 1.0);
	if(sunLight.iSkybox == 1)
		fMult = sunLight.fAmbientIntensity;
	vec4 vDirLightColor = vec4(sunLight.vColor*fMult, 1.0);
	*/
	//outputColor = vec4(texCoord.s, texCoord.t, 0, 1.0);
	//outputColor = vTexColor ;
	//outputColor = vec4(color, 1.0);
	//outputColor = vColor;
	outputColor = vec4(1, 0, 0, 1);
}