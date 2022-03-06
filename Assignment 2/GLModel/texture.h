#ifndef _TEXTURE_H_
#define _TEXTURE_H_

#include <cstdio>

#include "shared/opengl.h"

#include "shared/defs.h"

#include <util/util.h>

/*
Usage:
	(create/load texture)
	Texture* tex = new Texture("foo.bmp",true);

	(values can be any from the list at the end of the file)
	tex->ChangeSettings(GL_LINEAR,GL_LINEAR_MIPMAP_LINEAR,GL_REPEAT,GL_REPEAT,GL_MODULATE);
	
	(if you have more than one texture and you need to flip between them)
	tex->MakeCurrent(&TextureName);

	make calls using texture here...
*/
typedef struct _ImageRec {
    unsigned short imagic;
    unsigned short type;
    unsigned short dim;
    unsigned short xsize, ysize, zsize;
    unsigned int min, max;
    unsigned int wasteBytes;
    char name[80];
    unsigned long colorMap;
    FILE* file;
    unsigned char *tmp, *tmpR, *tmpG, *tmpB;
    unsigned long rleEnd;
    unsigned int *rowStart;
    int *rowSize;
} ImageRec;

class Texture
{
private:
	GLuint m_tex;

	void bwtorgba(unsigned char *b,unsigned char *l,int n);
	void latorgba(unsigned char *b, unsigned char *a,unsigned char *l,int n);
	void rgbtorgba(unsigned char *r,unsigned char *g,unsigned char *b,unsigned char *l,int n);
	void rgbatorgba(unsigned char *r,unsigned char *g,unsigned char *b,unsigned char *a,unsigned char *l,int n);
	void ConvertShort(unsigned short *array, long length);
	void ConvertLong(unsigned *array, long length);
	ImageRec *ImageOpen(char *fileName);
	void ImageClose(ImageRec *image);
	void ImageGetRow(ImageRec *image, unsigned char *buf, int y, int z);

	unsigned *rgb_read_texture(const char *name, int *width, int *height, int *components);

public:
	Texture(){}
	Texture(const char* fname,	bool make_mipmaps=true);
	~Texture();

	void loadBMP(const char* fname, bool make_mipmaps);
	void loadTGA(const char* fname, bool make_mipmaps);
	void loadRGB(const char* fname, bool make_mipmaps);
	void loadPPM(const char* fname, bool make_mipmaps);

	void ChangeSettings(GLuint mag_filter, 
		GLuint min_filter,
		GLuint wrap_s,
		GLuint wrap_t,
		GLuint env_mode);
	void MakeCurrent();
};

/*
 Parameters for ChangeSettings:
 mag_filter
    GL_LINEAR
	GL_NEAREST

 min_filter
	GL_LINEAR_MIPMAP_LINEAR
	GL_LINEAR_MIPMAP_NEAREST
	GL_LINEAR
	GL_NEAREST_MIPMAP_LINEAR
	GL_NEAREST_MIPMAP_NEAREST
	GL_NEAREST

 wrap_s
	GL_REPEAT
	GL_CLAMP
	
 wrap_t
	GL_REPEAT
	GL_CLAMP

 env_mode
	GL_BLEND
	GL_DECAL
	GL_MODULATE
	GL_REPLACE
*/

#endif