#pragma once

#include "simpleScene.h"
#include <ft2build.h>
#include FT_FREETYPE_H


class Text
{
public:
	Text();
	~Text();

	int init();
	void initTexture(int textureIndex, int iTextureLoc, int iAttributeCoord);
	void render_text(const char *text, float x, float y, float sx, float sy);
	void Text::bindBuffer();
private:
	FT_Library ft;
	FT_Face face;
	FT_GlyphSlot g;
	GLuint tex;
	GLuint vbo;


};

