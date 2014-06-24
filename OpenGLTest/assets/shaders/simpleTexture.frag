#version 330

// output data
layout (location = 0) out vec4 color;

uniform sampler2D tex;

in vec2 texCoord;

// debug
uniform bool debug;

vec4 red = vec4(1, 0, 0, 1);
vec4 orange = vec4(0.85, 0.64, 0.12, 1);
vec4 green = vec4(0, 1, 0, 1);
vec4 blue  = vec4(0, 0, 1, 1);
vec4 magenta  = vec4(1, 0, 1, 1);



void main()
{
	float depth = gl_FragCoord.z / gl_FragCoord.w;

	/*if(debug && depth < 0.5000000298022999)
	{
		color = red;
	}*/
	/*if(debug && depth < 1)
	{
		if(depth > 0.9)
			color = blue;
		else if(depth > 0.85)
			// ADD DEPTH DEBUG
			color = magenta;
		else 
			color = red;
		}
		else if(debug && depth > 1.1)
		{
			// ADD DEPTH DEBUG
			color = green;
		}
		else color = texture(tex, texCoord);
	}
	else color = texture(tex, texCoord);*/

	color = texture(tex, texCoord);
	//color = vec4(texCoord.s, texCoord.t, 0, 1.0);
}