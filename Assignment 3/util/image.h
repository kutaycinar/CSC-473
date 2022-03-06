#ifndef _IMAGE_H_
#define _IMAGE_H_

#ifdef WIN32
#include <windows.h>
#endif

#include "shared/opengl.h"

class GL_Image2D {
public:
  GL_Image2D() { m_hasAlpha = 0 ; m_pixelSize = 3 ; m_width = 0 ; 
					m_height = 0 ; m_data = NULL ;} ;
  GL_Image2D(const char *fname) { Read(fname) ; } ;
  GL_Image2D(int width, int height ) ;
  ~GL_Image2D() { if (m_data) delete []m_data; m_data = NULL ;} ;
	void Clear() { if (m_data) delete []m_data; m_data = NULL ; m_width = 0; m_height = 0; } ;

  int m_width ;
  int m_height ;
  char m_fname[132] ;

  GLubyte *m_data ;
  
  int m_hasAlpha ; // does it have alpha channel
  int m_pixelSize ; // pixel size in bytes

  int Read(const char *fname) ;
  int ReadPPM(const char *fname) ;
  int ReadBMP(const char *fname) ;
  int Write( FILE *fp ){ return 0;} ;
  int Write( const char *fname) { return 0;};

  int SetAlpha(double alpha) ;

  void Draw(GLdouble x = 0, GLdouble  y = 0, GLdouble  z = 0.0, GLdouble w = 1.0 ) ;
} ; 


#endif




