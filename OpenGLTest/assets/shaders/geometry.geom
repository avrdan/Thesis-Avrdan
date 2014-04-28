#version 330

layout (triangles) in;
layout (triangle_strip, max_vertices = 9) out;


// input variable is an array
// one value for each vertex
in vec2 vTexCoordPass[];
in vec3 vNormalPass[];

// out variables
smooth out vec3 vNormal;
smooth out vec2 vTexCoord;

// defines bending of the triangle centroid
uniform float fBender;

uniform struct Matrices
{
	mat4 projMatrix;
	mat4 modelMatrix;
	mat4 viewMatrix;
	mat4 normalMatrix;
} matrices;

void main()
{
	// calculate mvp
	mat4 mMVP = matrices.projMatrix * matrices.viewMatrix * matrices.modelMatrix;

	// calculate the centroid point
	// also add bending
	vec3 vMiddle = (gl_in[0].gl_Position.xyz + gl_in[1].gl_Position.xyz + gl_in[2].gl_Position.xyz)/3.0 +
				(vNormalPass[0] + vNormalPass[1] + vNormalPass[2]) * fBender;

	// uv for centroid
	vec2 vTexCoordMiddle = (vTexCoordPass[0] + vTexCoordPass[1] + vTexCoordPass[2]) / 3.0;

	// transform normals
	vec3 vNormalTransformed[3];
	for (int i = 0; i < 3; i++)
	{
		vNormalTransformed[i] = (vec4(vNormalPass[i], 1.0) * matrices.normalMatrix).xyz;
	}

	// calculate centroid normal
	vec3 vNormalMiddle = (vNormalTransformed[0] + vNormalTransformed[1] + vNormalTransformed[2]) / 3.0;

	for (int i = 0; i < 3; i++)
	{
		// emit first vertex
		vec3 vPos   = gl_in[i].gl_Position.xyz;
		gl_Position = mMVP * vec4(vPos, 1.0);
		vNormal = (vec4(vNormalTransformed[i], 1.0)).xyz;
		vTexCoord = vTexCoordPass[i];
		EmitVertex();
		
		// emit the second vertex
		vPos = gl_in[(i+1)%3].gl_Position.xyz;
		gl_Position = mMVP * vec4(vPos, 1.0);
		vNormal = (vec4(vNormalTransformed[(i+1)%3], 1.0)).xyz;
		vTexCoord = vTexCoordPass[(i+1)%3];
		EmitVertex();

		// emit third vertex - centroid
		gl_Position = mMVP * vec4(vMiddle, 1.0);
		vNormal		= vNormalMiddle;
		vTexCoord	= vTexCoordMiddle;
		EmitVertex();

		EndPrimitive();
	}

}