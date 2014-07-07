#version 330

in vec2 texcoord;
uniform sampler2D tex;
uniform vec4 color;
 
out vec4 outputColor;

void main(void) {
  outputColor = vec4(1, 1, 1, texture(tex, texcoord).a) * color;
}