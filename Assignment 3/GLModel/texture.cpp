/* #ifdef WIN32
#include <windows.h>
#endif

#include <cmath>
#include <string.h>
#include <fstream>

#include "texture.h"
#include "GLModel.h"
#include "util/util.h"
*/

#ifdef WIN32
#include <windows.h>
#endif


#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <string.h>
#include <fstream>

#include "shared/opengl.h"

#include "shared/defs.h"

#include "util/util.h"
#include "GLModel/GLModel.h"



#include "texture.h"

#ifdef __APPLE__
#define _strlwr strlwr
#endif

#ifndef BYTE
#define BYTE char
#endif

Texture::Texture(const char* fname, bool make_mipmaps)
{
	m_tex = 0;
	char ext[4];
	strcpy(ext,fname+strlen(fname)-3);
	//_strlwr(ext);
	ext[0] = tolower(ext[0]) ;
	ext[1] = tolower(ext[1]) ;
	ext[2] = tolower(ext[2]) ;
	
	if(strcmp(ext,"bmp")==0)
		loadBMP(fname,make_mipmaps);
	else if(strcmp(ext,"tga")==0)
		loadTGA(fname,make_mipmaps);
	else if(strcmp(ext,"rgb")==0)
		loadRGB(fname,make_mipmaps);
	else if(strcmp(ext,"ppm")==0)
		loadPPM(fname,make_mipmaps);
	else
		// animTcl::OutputMessage("ERROR: Texure::Texture: Unknown image type.");
		return ;
}

void Texture::loadBMP(const char *fname,
							bool make_mipmaps)
{
	m_tex = 0;
	//animTcl::OutputMessage("ERROR: Texture::loadBMP not implemented yet") ;
	return ;


	GL_Image2D image ;
	if( image.Read((const char*) fname) <= 0 )
	{
		//animTcl::OutputMessage("ERROR: Cannot load BMP file.");
		return;
	}

	glGenTextures(1, &m_tex);
	glBindTexture(GL_TEXTURE_2D, m_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	glPixelStorei(GL_UNPACK_ALIGNMENT,1);

	if(make_mipmaps) 
	{
		if(gluBuild2DMipmaps(GL_TEXTURE_2D, 3, image.m_width, image.m_height, 
							 GL_RGB, GL_UNSIGNED_BYTE, image.m_data)!=0)
			return ; //animTcl::OutputMessage((char *) gluErrorString(glGetError()));
	}
	else 
	{
		glTexImage2D(GL_TEXTURE_2D, 0, 3, image.m_width, image.m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, image.m_data);
	}

	// deallocate Image
	image.Clear() ;
}

void Texture::loadTGA(const char* szFileName, bool make_mipmaps)
{
	// TGA file offsets
	const std::istream::off_type TGA_IMAGEIDLENGTH_OFFSET = 0;
	const std::istream::off_type TGA_COLORMAPTYPE_OFFSET = 1;
	const std::istream::off_type TGA_IMAGETYPE_OFFSET = 2;
	const std::istream::off_type TGA_COLORMAPLENGTH_OFFSET = 5;
	const std::istream::off_type TGA_COLORMAPENTRYSIZE_OFFSET = 7;
	const std::istream::off_type TGA_IMAGEWIDTH_OFFSET = 12;
	const std::istream::off_type TGA_IMAGEHEIGHT_OFFSET = 14;
	const std::istream::off_type TGA_IMAGEBITDEPTH_OFFSET = 16;
	const std::istream::off_type TGA_IMAGEID_OFFSET = 18;
	// TGA file modes
	const unsigned char TGA_IMAGETYPE_RGBRAW = 2;
	// File input stream
	std::ifstream ifsTextureFile(szFileName, std::ios::binary);

	// Read TGA identification field length
	unsigned char uchTGAIDLength;
	ifsTextureFile.read(reinterpret_cast<char*>(&uchTGAIDLength), 1);
	
	// Read TGA Color Map type
	unsigned char uchTGAColorMapType;
	ifsTextureFile.read(reinterpret_cast<char*>(&uchTGAColorMapType), 1);
	
	// Read TGA file type
	unsigned char uchTGAType;
	ifsTextureFile.read(reinterpret_cast<char*>(&uchTGAType), 1);

	if (uchTGAType != TGA_IMAGETYPE_RGBRAW)
	{
		// Only raw RGB format is supported
		//animTcl::OutputMessage("ERROR: Only raw RGB TGA supported.");
		return;
	}

	// Read TGA Color Map Info
	unsigned short nTGAColorMapLength;
	unsigned short nTGAColorMapEntrySize;
	ifsTextureFile.seekg(TGA_COLORMAPLENGTH_OFFSET, std::ios::beg);
	ifsTextureFile.read(reinterpret_cast<char*>(&nTGAColorMapLength), 2);
	ifsTextureFile.read(reinterpret_cast<char*>(&nTGAColorMapEntrySize), 2);

	// Read TGA Image Dimensions
	unsigned short nTGAImageWidth;
	unsigned short nTGAImageHeight;
	unsigned char nTGAImageBitDepth;

	ifsTextureFile.seekg(TGA_IMAGEWIDTH_OFFSET, std::ios::beg);
	ifsTextureFile.read(reinterpret_cast<char*>(&nTGAImageWidth), 2);
	ifsTextureFile.read(reinterpret_cast<char*>(&nTGAImageHeight), 2);
	ifsTextureFile.read(reinterpret_cast<char*>(&nTGAImageBitDepth), 1);

	// Skip Image ID and Color Map
	ifsTextureFile.seekg(TGA_IMAGEID_OFFSET + uchTGAIDLength + 
		(nTGAColorMapLength * (nTGAColorMapEntrySize >> 3)),
		std::ios::beg);

	// Calculate required memory amount
	unsigned long ulImageSize = nTGAImageWidth * nTGAImageHeight * (nTGAImageBitDepth >> 3);

	// Allocate memory for texture
	unsigned char * puchTextureData = new unsigned char[ulImageSize]; 

	// Read image bytes into memory
	ifsTextureFile.read(reinterpret_cast<char*>(puchTextureData), ulImageSize); 

	glGenTextures(1, &m_tex);
	glBindTexture(GL_TEXTURE_2D, m_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	if(make_mipmaps)
	{
		switch(nTGAImageBitDepth)
		{
			case 24:
				gluBuild2DMipmaps(GL_TEXTURE_2D, 3, nTGAImageWidth, nTGAImageHeight, GL_BGR_EXT, GL_UNSIGNED_BYTE, puchTextureData);
				break;
			case 32:
				gluBuild2DMipmaps(GL_TEXTURE_2D, 4, nTGAImageWidth, nTGAImageHeight, GL_BGRA_EXT, GL_UNSIGNED_BYTE, puchTextureData);
				break;
		}
	}
	else
	{
		switch(nTGAImageBitDepth)
		{
			case 24:
				glTexImage2D(GL_TEXTURE_2D, 0, 3, nTGAImageWidth, nTGAImageHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE,	puchTextureData);
				break;
			case 32:
				glTexImage2D(GL_TEXTURE_2D, 0, 4, nTGAImageWidth, nTGAImageHeight, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, puchTextureData);
				break;
		}
	}

	if(puchTextureData) delete[] puchTextureData;
}

void Texture::loadRGB(const char* fn, bool make_mipmaps)
{
	unsigned int *image = NULL;
	int width, height, depth;

	m_tex = 0;
	image = rgb_read_texture(fn, &width, &height, &depth);
	if (image == NULL)
	{
		// animTcl::OutputMessage("ERROR: Cannot open rgb file.");
		return;
	}

	glGenTextures(1, &m_tex);
	glBindTexture(GL_TEXTURE_2D, m_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	if(make_mipmaps)
		gluBuild2DMipmaps(GL_TEXTURE_2D, depth, width, height, GL_RGBA, GL_UNSIGNED_BYTE, image);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, depth, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);

	if(image) free(image);
}

void Texture::loadPPM(const char* fn, bool make_mipmaps)
{
	int height, width;
	unsigned char* image = NULL;

	m_tex = 0;
    image = glmReadPPM(fn, &width, &height);
    if (!image)
	{
		// animTcl::OutputMessage("ERROR: Cannot open ppm file.");
		return;
	}

	glGenTextures(1, &m_tex);
	glBindTexture(GL_TEXTURE_2D, m_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);

	if(make_mipmaps)
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);

	if(image) free(image);
}

void Texture::ChangeSettings(GLuint mag_filter,
							 GLuint min_filter,
							 GLuint wrap_s,
							 GLuint wrap_t,
							 GLuint env_mode)
{
	glBindTexture(GL_TEXTURE_2D, m_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap_s);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap_t);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, (GLfloat) env_mode);
}

void Texture::MakeCurrent()
{
	glBindTexture(GL_TEXTURE_2D, m_tex);
}

Texture::~Texture()
{
	glDeleteTextures(1,&m_tex);
}

void Texture::bwtorgba(unsigned char *b,unsigned char *l,int n)
{
    while(n--) {
	l[0] = *b;
	l[1] = *b;
	l[2] = *b;
	l[3] = 0xff;
	l += 4; b++;
    }
}

void Texture::latorgba(unsigned char *b, unsigned char *a,unsigned char *l,int n)
{
    while(n--) {
	l[0] = *b;
	l[1] = *b;
	l[2] = *b;
	l[3] = *a;
	l += 4; b++; a++;
    }
}

void Texture::rgbtorgba(unsigned char *r,unsigned char *g,unsigned char *b,unsigned char *l,int n)
{
    while(n--) {
	l[0] = r[0];
	l[1] = g[0];
	l[2] = b[0];
	l[3] = 0xff;
	l += 4; r++; g++; b++;
    }
}

void Texture::rgbatorgba(unsigned char *r,unsigned char *g,unsigned char *b,unsigned char *a,unsigned char *l,int n)
{
    while(n--) {
	l[0] = r[0];
	l[1] = g[0];
	l[2] = b[0];
	l[3] = a[0];
        l += 4; r++; g++; b++; a++;
    }
}

void Texture::ConvertShort(unsigned short *array, long length) 
{
    unsigned b1, b2;
    unsigned char *ptr;

    ptr = (unsigned char *)array;
    while (length--) {
	b1 = *ptr++;
	b2 = *ptr++;
	*array++ = (b1 << 8) | (b2);
    }
}

void Texture::ConvertLong(unsigned *array, long length)
{
    unsigned long b1, b2, b3, b4;
    unsigned char *ptr;

    ptr = (unsigned char *)array;
    while (length--) {
	b1 = *ptr++;
	b2 = *ptr++;
	b3 = *ptr++;
	b4 = *ptr++;
	*array++ = (b1 << 24) | (b2 << 16) | (b3 << 8) | (b4);
    }
}

ImageRec* Texture::ImageOpen(char *fileName)
{
    union {
	int testWord;
	char testByte[4];
    } endianTest;
    ImageRec *image;
    int swapFlag;
    int x;

    endianTest.testWord = 1;
    if (endianTest.testByte[0] == 1) {
	swapFlag = 1;
    } else {
	swapFlag = 0;
    }

    image = (ImageRec *)malloc(sizeof(ImageRec));
    if (image == NULL) {
	fprintf(stderr, "Out of memory!\n");
	exit(1);
    }
    if ((image->file = fopen(fileName, "rb")) == NULL) {
	perror(fileName);
	exit(1);
    }

    fread(image, 1, 12, image->file);

    if (swapFlag) {
	ConvertShort(&image->imagic, 6);
    }

    image->tmp = (unsigned char *)malloc(image->xsize*256);
    image->tmpR = (unsigned char *)malloc(image->xsize*256);
    image->tmpG = (unsigned char *)malloc(image->xsize*256);
    image->tmpB = (unsigned char *)malloc(image->xsize*256);
    if (image->tmp == NULL || image->tmpR == NULL || image->tmpG == NULL ||
	image->tmpB == NULL) {
	fprintf(stderr, "Out of memory!\n");
	exit(1);
    }

    if ((image->type & 0xFF00) == 0x0100) {
	x = image->ysize * image->zsize * sizeof(unsigned);
	image->rowStart = (unsigned *)malloc(x);
	image->rowSize = (int *)malloc(x);
	if (image->rowStart == NULL || image->rowSize == NULL) {
	    fprintf(stderr, "Out of memory!\n");
	    exit(1);
	}
	image->rleEnd = 512 + (2 * x);
	fseek(image->file, 512, SEEK_SET);
	fread(image->rowStart, 1, x, image->file);
	fread(image->rowSize, 1, x, image->file);
	if (swapFlag) {
	    ConvertLong(image->rowStart, x/sizeof(unsigned));
	    ConvertLong((unsigned *)image->rowSize, x/sizeof(int));
	}
    }
    return image;
}

void Texture::ImageClose(ImageRec *image)
{
    fclose(image->file);
    free(image->tmp);
    free(image->tmpR);
    free(image->tmpG);
    free(image->tmpB);
    free(image);
}

void Texture::ImageGetRow(ImageRec *image, unsigned char *buf, int y, int z)
{
    unsigned char *iPtr, *oPtr, pixel;
    int count;

    if ((image->type & 0xFF00) == 0x0100) {
	fseek(image->file, image->rowStart[y+z*image->ysize], SEEK_SET);
	fread(image->tmp, 1, (unsigned int)image->rowSize[y+z*image->ysize],
	      image->file);

	iPtr = image->tmp;
	oPtr = buf;
	while (1) {
	    pixel = *iPtr++;
	    count = (int)(pixel & 0x7F);
	    if (!count) {
		return;
	    }
	    if (pixel & 0x80) {
		while (count--) {
		    *oPtr++ = *iPtr++;
		}
	    } else {
		pixel = *iPtr++;
		while (count--) {
		    *oPtr++ = pixel;
		}
	    }
	}
    } else {
	fseek(image->file, 512+(y*image->xsize)+(z*image->xsize*image->ysize),
	      SEEK_SET);
	fread(buf, 1, image->xsize, image->file);
    }
}

unsigned* Texture::rgb_read_texture(const char *name, int *width, int *height, int *components)
{
    unsigned *base, *lptr;
    unsigned char *rbuf, *gbuf, *bbuf, *abuf;
    ImageRec *image;
    int y;

	char my_name[50];

	strcpy( my_name, name );

    image = ImageOpen(my_name);
    
    if(!image)
	return NULL;
    (*width)=image->xsize;
    (*height)=image->ysize;
    (*components)=image->zsize;
    base = (unsigned *)malloc(image->xsize*image->ysize*sizeof(unsigned));
    rbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    gbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    bbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    abuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    if(!base || !rbuf || !gbuf || !bbuf)
      return NULL;
    lptr = base;
    for(y=0; y<image->ysize; y++) {
	if(image->zsize>=4) {
	    ImageGetRow(image,rbuf,y,0);
	    ImageGetRow(image,gbuf,y,1);
	    ImageGetRow(image,bbuf,y,2);
	    ImageGetRow(image,abuf,y,3);
	    rgbatorgba(rbuf,gbuf,bbuf,abuf,(unsigned char *)lptr,image->xsize);
	    lptr += image->xsize;
	} else if(image->zsize==3) {
	    ImageGetRow(image,rbuf,y,0);
	    ImageGetRow(image,gbuf,y,1);
	    ImageGetRow(image,bbuf,y,2);
	    rgbtorgba(rbuf,gbuf,bbuf,(unsigned char *)lptr,image->xsize);
	    lptr += image->xsize;
	} else if(image->zsize==2) {
	    ImageGetRow(image,rbuf,y,0);
	    ImageGetRow(image,abuf,y,1);
	    latorgba(rbuf,abuf,(unsigned char *)lptr,image->xsize);
	    lptr += image->xsize;
	} else {
	    ImageGetRow(image,rbuf,y,0);
	    bwtorgba(rbuf,(unsigned char *)lptr,image->xsize);
	    lptr += image->xsize;
	}
    }
    ImageClose(image);
    free(rbuf);
    free(gbuf);
    free(bbuf);
    free(abuf);

    return (unsigned *) base;
}


