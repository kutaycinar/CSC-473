
#include <shared/defs.h>
#include "anim.h"
#include "animTcl.h"

void mainLoop(void) ;
extern SETVAR myScriptVariables[] ;
extern void mySetScriptCommands(Tcl_Interp *interp) ;

#include <stdlib.h>
#include <assert.h>

int animTcl::UsingTk =	FALSE;

int garbage = 10 ;

Tcl_Interp *animTcl::m_interpreter = NULL;

SETVAR animTcl::setlist[] = {
	"garbage", TCL_LINK_INT, (char *) &garbage,
		"",0,(char *) NULL 
};

// PringeMessage: This is the implementation of the generic 
// message printing routine. In this case it wrapps animTcl::OutputMessage
void PrintMessage(char *format, ...)
{
	va_list vl;

	//  format is the last argument specified; all
	//  others must be accessed using the variable-argument macros.
	va_start( vl, format);   
	static char message[1024];
	static char message2[1024];

	vsprintf(message,format,vl);
	va_end( vl );

	int len = (int) strlen(message);
	//assert( len < 1024); // Exceeded internal print buffer.
	if (len>= 1024) {
		strncpy(message2, message, 1022);
		strcpy(message, message2);
	}
	animTcl::OutputMessage(message) ;
}


// InitUserTclCommands:
//	Initializes user-defined Tcl Commands
void animTcl::InitUserTclCommands(Tcl_Interp *interp)
{
	Tcl_CreateCommand(interp, "system", Command, (ClientData) NULL,
		(Tcl_CmdDeleteProc *)	NULL);
	Tcl_CreateCommand(interp, "simulator", Command, (ClientData) NULL,
		(Tcl_CmdDeleteProc *)	NULL);
	Tcl_CreateCommand(interp, "show", Command, (ClientData) NULL,
		(Tcl_CmdDeleteProc *)	NULL);
	mySetScriptCommands(interp) ;
}


// checkEvents:
//	Processes both Tk and TCL events.
//
void animTcl::checkEvents(void)
{
	if(	UsingTk	== FALSE)
		while(Tcl_DoOneEvent(TCL_DONT_WAIT) != 0) ;
	else
		while(Tk_DoOneEvent(TK_DONT_WAIT) != 0) ;
}


// InitTclTk:
//	     does Tcl/Tk setup,
//	     hands over	control	to a TK	Mainloop,
//	    
void animTcl::InitTclTk(int argc, char	**argv)
{

	if (FALSE)
	{
		UsingTk	= FALSE	;
		Tcl_Main(argc, argv, Tcl_AppInit);
	}
	else
	{
		UsingTk	= TRUE ;
		Tk_Main(argc, argv, Tcl_AppInit);
	}

}

// Tcl_AppInit:
//	application-specific initialization
int animTcl::Tcl_AppInit(Tcl_Interp *interp)
{
	if (Tcl_Init(interp) == TCL_ERROR) return TCL_ERROR;

	// Set the interpreter variable immediately
	// because it is used in outputMessage()
	m_interpreter = interp;

	LinkUserVariables(interp) ; 
	InitUserTclCommands(interp);

	char filename[256];
	if (UsingTk	== TRUE)
	{
		if (Tk_Init(interp) == TCL_ERROR) 
			return TCL_ERROR;

		//sprintf(filename,"%s/shell.tcl",getenv("ANIM_DIR"));
#ifdef __APPLE__
     	sprintf(filename,"./shell_mac.tcl");
#else
		sprintf(filename,"./shell.tcl");
#endif
		if (TCL_OK != Tcl_EvalFile(interp,filename))
			animTcl::OutputMessage("error in script: %s",	filename);

	}
	// read the start up file

	sprintf(filename,"start.tcl");


	if (TCL_OK != Tcl_EvalFile(interp,filename) )
		animTcl::OutputMessage("error in script: %s", filename);

	// now start mainLoop
	mainLoop() ;
	return TCL_OK;
}

// LinkUserVariables:
//	Links user variables
void animTcl::LinkUserVariables(Tcl_Interp *interp)
{
	int	count =	0;
	for(int i =	0 ; setlist[i].ptr ; i++ )
	{
		SETVAR *v = &setlist[i]	;

		if ( Tcl_LinkVar(interp,v->name, v->ptr,v->type) == TCL_ERROR) {
			animTcl::OutputMessage("ERROR: Cannot link	variables %s", v->name);
		}
		else
			count++;
	}
	count = 0 ;
	for(int i =	0 ; setlist[i].ptr ; i++ )
	{
		SETVAR *v = &myScriptVariables[i];

		if ( Tcl_LinkVar(interp,v->name, v->ptr,v->type) == TCL_ERROR) {
			animTcl::OutputMessage("ERROR: Cannot link	variables %s", v->name);
		}
		else
			count++;
	}

	animTcl::OutputMessage("Linked %d user Tcl	variables.\n",count);
}



// quit
//	Quits the application.
//
int animTcl::quit(ClientData clientData, Tcl_Interp *interp, int argc,	myCONST_SPEC char **argv)
{
	// Deallocate Tcl properly to close off GUI elements, otherwise on some
	// machines, we may get an invalid memory access error.
	ExecuteCommand("exit"); 
	exit(0);
	return(0);
}

void animTcl::OutputResult(char *format, ...)
{
	va_list vl;

	//  format is the last argument specified; all
	//  others must be accessed using the variable-argument macros.
	va_start( vl, format);   
	static char message[1024];
	vsprintf(message,format,vl);
	va_end( vl );

	int len = (int) strlen(message);
	assert( len < 1024); // Exceeded internal print buffer.

	Tcl_AppendResult(m_interpreter,message,NULL);

}

void traceProc(ClientData clientDatat, Tcl_Interp *interp,
			   int level, char *command, Tcl_CmdProc *cmdProc,
			   ClientData cmdClientData, int argc, char *argv[]) 
{
	return ;
}

void animTcl::OutputMessage(char *format, ...)
{
	va_list vl;   

	//  format is the last argument specified; all
	//  others must be accessed using the variable-argument macros.
	va_start( vl, format);   
	static char message[1024];
	static char message2[1024];

	vsprintf(message,format,vl);
	va_end( vl );

	int len = (int) strlen(message);
	//assert( len < 1024); // Exceeded internal print buffer.
	if (len>= 1024) {
		strncpy(message2, message, 1022);
		strcpy(message, message2);
	}

	char command[1050]; 

	// use lists etc to prevent the interpreter from
	// substituting variables and interpreting special characters
	sprintf(command,"puts [lindex [list {%s}] 0]",message);

	//	 replace new lines with space
	char *s = &command[0] ;
	for( ; *s != '\0' ; s++ )
		if( *s == '\n' ) *s = 32 ;

	//	Tcl_Trace trace = Tcl_CreateTrace(Interpreter,3, traceProc, NULL ) ;
	if ( m_interpreter )
		Tcl_Eval(m_interpreter, command) ;
	//	Tcl_DeleteTrace(Interpreter, trace) ;
}



// Function name	: animTcl::ExecuteCommand
// Description	    : 
// Return type		: void 
// Argument         : char *command
void animTcl::ExecuteCommand(char *command)
{
	Tcl_Eval(m_interpreter,command);
}


// OutputListElement:
//	Adds an item to the result list.
void animTcl::OutputListElement(char * format, ...)
{
	va_list vl;

	//  format is the last argument specified; all
	//  others must be accessed using the variable-argument macros.
	va_start( vl, format);   
	static char message[1024];
	vsprintf(message,format,vl);
	va_end( vl );

	int len = (int) strlen(message);
	assert( len < 1024); // Exceeded internal print buffer.

	Tcl_AppendElement(m_interpreter,message);
}

int animTcl::Command(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	if (argc < 2 ) {
		animTcl::OutputMessage("Usage: system|simulator <name> ...")  ;
		return  TCL_ERROR;
	}
	else if( strcmp(argv[0], "system") == 0 )
	{

		// get the requested system by name and dispatch the command

		BaseSystem* tempSystem =  
			GlobalResourceManager::use()->getSystem( argv[1] );
			
		if( tempSystem != NULL )
		{
			tempSystem->command( 
			argc - 2, &argv[ 2 ] );

		}
		else
		{

			animTcl::OutputMessage( "system %s was not found", argv[ 1 ] )  ;
			return  TCL_ERROR;

		}

	}
	else if( strcmp(argv[0], "simulator") == 0 )
	{

		// get the requested simulator by name and dispatch the command

		BaseSimulator* tempSimulator = 
			GlobalResourceManager::use()->getSimulator( argv[1] );
			
		if( tempSimulator != NULL )
		{
			tempSimulator->command( 
			argc - 2, &argv[ 2 ] );

		}
		else
		{

			animTcl::OutputMessage("simulator %s was not found", argv[ 1 ] )  ;
			return  TCL_ERROR;

		}

	}
	else if( strcmp(argv[0], "show" ) == 0 )
	{
		if( argc < 2 ) 
		{
			animTcl::OutputMessage("syntax: show system|simulator") ;
			return TCL_OK ;
		}

		if( strcmp(argv[1], "simulator" ) == 0 ) 
		{

			unsigned int index;

			for( index = 0; index < 
				GlobalResourceManager::use()->getNumberOfSimulators(); index++ )
			{

			std::string name;

			GlobalResourceManager::use()->getSimulatorFromIndex( 
				index )->getName( name );

			animTcl::OutputListElement("%s", name.c_str() ) ;

			}

		}
		else if( strcmp(argv[1], "system" ) == 0 ) 
		{

			unsigned int index;

			for( index = 0; index < 
				GlobalResourceManager::use()->getNumberOfSystems(); index++ )
			{

			std::string name;

			GlobalResourceManager::use()->getSystemFromIndex( 
				index )->getName( name );

			animTcl::OutputListElement("%s", name.c_str() ) ;

			}

		}
	}
	else if( strcmp(argv[0], "reset") == 0 )
	{

		GlobalResourceManager::use()->resetAll();

	}

	//PostRedisplay();

	return TCL_OK;

}

