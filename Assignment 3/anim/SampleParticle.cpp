#include "SampleParticle.h"

SampleParticle::SampleParticle( const std::string& name ):
	BaseSystem( name ),
	m_sx( 1.0f ),
	m_sy( 1.0f ),
	m_sz( 1.0f )
{ 

	setVector( m_pos, 0, 0, 0 );
	
}	// SampleParticle

void SampleParticle::getState( double* p )
{ 

	VecCopy( p, m_pos ); 

}	// SampleParticle::getState

void SampleParticle::setState( double  *p )
{ 

	VecCopy(m_pos,p); 

}	// SampleParticle::setState

void SampleParticle::reset( double time ) 
{ 

	setVector( m_pos, 0, 0, 0 ); 
	
}	// SampleParticle::Reset


int SampleParticle::command(int argc, myCONST_SPEC char **argv) 
{
	if( argc < 1 )
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str()) ;
		return TCL_ERROR ;
	}
	else if( strcmp(argv[0], "read") == 0 )
	{
		if( argc == 2 )
		{
			m_model.ReadOBJ(argv[1]) ;
			glmFacetNormals(&m_model) ;
			glmVertexNormals(&m_model, 90) ;
			return TCL_OK ;
		}
		else 
		{
			animTcl::OutputMessage("Usage: read <file_name>") ;
			return TCL_ERROR ;
		}
	}
	else if( strcmp(argv[0], "scale") == 0 )
	{
		if( argc == 4 )
		{
			m_sx = (float)atof(argv[1]) ;
			m_sy = (float)atof(argv[2]) ;
			m_sz = (float)atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: scale <sx> <sy> <sz> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "pos") == 0 )
	{
		if( argc == 4 )
		{
			m_pos[0] = atof(argv[1]) ;
			m_pos[1] = atof(argv[2]) ;
			m_pos[2] = atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: pos <x> <y> <z> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "flipNormals") == 0 )
	{
		flipNormals() ;
		return TCL_OK ;
		
	}
	else if( strcmp(argv[0], "reset") == 0)
	{
		double p[3] = {0,0,0} ;
		setState(p) ;
	}
    
    glutPostRedisplay() ;
	return TCL_OK ;

}	// SampleParticle::command

void SampleParticle::display( GLenum mode ) 
{
	glEnable(GL_LIGHTING) ;
	glMatrixMode(GL_MODELVIEW) ;
	glPushMatrix() ;
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(m_pos[0],m_pos[1],m_pos[2]) ;
	glScalef(m_sx,m_sy,m_sz) ;

	if( m_model.numvertices > 0 )
		glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	else
		glutSolidSphere(1.0,20,20) ;

	glPopMatrix();
	glPopAttrib();

}	// SampleParticle::display
