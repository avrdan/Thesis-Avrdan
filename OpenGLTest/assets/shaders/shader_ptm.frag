#version 330

uniform sampler2D projMap;
varying vec3 normal, lightDir, eyeVec;

void main (void)
{
	vec4 final_color = 
		(gl_FrontLightModelProduct.sceneColor * gl_FrontMaterial.ambient) +
		(gl_LightSource[0].ambient * gl_FrontMaterial.ambient);

	vec3 N = normalize(normal);
	vec3 L = normalize(lightDir);

	float lambertTerm = dot(N, L);

	if (lambertTerm > 0.0)
	{
		final_color += gl_LightSource[0].diffuse * gl_FrontMaterial.diffuse * lambertTerm;
		vec3 E = normalize(eyeVec);
		vec3 R = reflect(-L, N);
		float specular = pow( max(dot(R, E), 0, 0), gl_FrontMaterial.shininess);
		final_color += gl_LightSource[0].specular * gl_FrontMaterial.specular * specular;
	
		// supress the reverse projection
		if (gl_TexCoord[0].q > 0.0)
		{
			// vec2 projCoords = gl_TexCoord[0].st / gl_TexCoord[0].q;
  			// vec4 ProjMapColor = texture2D(projMap, projCoords);
			vec4 ProjMapColor = texture2DProj(projMap, gl_TexCoord[0]);
			final_color += ProjMapColor*lambertTerm;
		}
	}

	gl_FragColor = final_color;
}