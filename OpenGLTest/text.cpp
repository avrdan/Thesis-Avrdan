#include "text.h"


Text::Text()
{
	
}

Text::~Text()
{
}

int Text::init()
{
	// init FreeType
	if (FT_Init_FreeType(&ft)) {
		fprintf(stderr, "Could not init freetype library\n");
		return 1;
	}

	// load font
	if (FT_New_Face(ft, "./assets/fonts/FreeSans.ttf", 0, &face)) {
		fprintf(stderr, "Could not open font\n");
		return 1;
	}

	// set font size
	FT_Set_Pixel_Sizes(face, 0, 48);

	// try to load a gliph
	if (FT_Load_Char(face, 'X', FT_LOAD_RENDER)) {
		fprintf(stderr, "Could not load character 'X'\n");
		return 1;
	}

	// shorthand
	g = face->glyph;
}

void Text::initTexture(int textureIndex, int iTextureLoc, int iAttributeCoord)
{
	
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(iTextureLoc, 0);

	// set up texture params
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// remove alignment restrictions
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glGenBuffers(1, &vbo);
	glEnableVertexAttribArray(iAttributeCoord);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexAttribPointer(iAttributeCoord, 4, GL_FLOAT, GL_FALSE, 0, 0);
}

void Text::bindBuffer()
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
}

void Text::render_text(const char *text, float x, float y, float sx, float sy) {
	const char *p;

	for (p = text; *p; p++) {
		if (FT_Load_Char(face, *p, FT_LOAD_RENDER))
			continue;

		glTexImage2D(
			GL_TEXTURE_2D,
			0,
			GL_ALPHA,
			g->bitmap.width,
			g->bitmap.rows,
			0,
			GL_ALPHA,
			GL_UNSIGNED_BYTE,
			g->bitmap.buffer
			);

		float x2 = x + g->bitmap_left * sx;
		float y2 = -y - g->bitmap_top * sy;
		float w = g->bitmap.width * sx;
		float h = g->bitmap.rows * sy;

		GLfloat box[4][4] = {
			{ x2, -y2, 0, 0 },
			{ x2 + w, -y2, 1, 0 },
			{ x2, -y2 - h, 0, 1 },
			{ x2 + w, -y2 - h, 1, 1 },
		};

		glBufferData(GL_ARRAY_BUFFER, sizeof box, box, GL_DYNAMIC_DRAW);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		x += (g->advance.x >> 6) * sx;
		y += (g->advance.y >> 6) * sy;
	}
}