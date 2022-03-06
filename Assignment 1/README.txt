------------------------------------
TEMPLATE CODE FOR CS 174C/CS 274C
------------------------------------

Prerequisites
--------------

- Windows XP or Vista
- Visual Studio 2005 / 2008
- For visual Studio 2013: 
"Image has safe exceptions handlers" setting to NO for "anim" project, it is under anim/Linker/All Options/.

Start as follows:

- Double click on cs174.sln
- Build the solution.
- Run it

If everything worked fine you should see two windows. The first window is an 
OpenGL window showing a soccer ball. The second window is a Tcl shell

If the program does not run, you can't see the soccer ball, or the shell window
does not show any prompt, then something went wrong and you should talk to the TA.

The executable is created in the "Build" folder. You can run the main program 
outside Visual Studio by running the file "animp.exe"

Interaction
------------

- Moving the mouse with the right button pressed rotates the scene.
- Moving the mouse with the middle button pressed zooms the scene

The following key strokes are recognized:

- r	restore the original view
- 0	reset all systems
- a	toggle animation
- s	toggle simulation
- m	toggle frame dumping
- t	reset all timers
- q	quit
- h	print these instructions

Code
------------

All classes that interact with the template must sub-class one the following
classed depending what they are meant for:
- BaseObject
- BaseSystem
- BaseSimulator

The two classes SampleParticle and SampleGravitySimulator are examples of how
 to implement a system or a simulator

In the file myScene.cpp look at the global function MakeScene for an example
on how to instantiate objects and register them with the global resource manager.
You should modify this function to instantiate your own classes.

in myScene.cpp the following three functions are used for I/O

- myKey		is triggered when the user presses a key
- myMouse	is triggered when a mouse button even occurs
- myMotion	is triggered when the user moves the mouse

NOTE: don't touch the callback functions in anim.cpp

Classes
------------

- BaseObject: is the base class for all objects that need to interact with the template
code. This is only used for inanimate objects, so you most likely do not need to inherit
from this one directly

- BaseSystem: Any object that can be animated needs to inherit from this one. You MUST
always overload the following two methods:
	* BaseSystem::getState
	* BaseSystem::setState

NOTE: you decide the size of the pointer p passed to these methods.

If you don't overload these methods correctly I made sure that an
 assertion failure is triggered at run-time

You must also overload the display method and put your own OpenGL code in it for
the system to be drawable.

- BaseSimulator: is the base class for classes that encapsulate the code that 
orchestrates the actual animation of other systems. You MUST overload the following
methods:
	* BaseSimulator::Step
	* BaseSimulator::init

Commands and the Shell
------------

The template code embeds a Tcl shell, which is very nice for debugging your programs.
This shell understands standard Tcl commands plus some additional commands that we 
specified

The follwing two commands are built-in in the template:

- "show system"				Shows a list of the names of all systems
instantiated in the scene

- "show simulator"			Shows a list of the names of all simulators
instantiated in the scene

- "system <name> <arg1> <arg2> ..."	Invokes system <name> with the given arguments

- "simulator <name> <arg1> <arg2> ..."	Invokes simulator <name> with the given arguments

Every time you invoke a system or a simulator from the shell, the "command" method of
the corresponding object is called. Obviously, you should implement the command method for
your own classes to interface with the shell!

Adding global commands
------------

If you want to add global commands for the shell you should do the following:

1) add a new global function with the same signature as the function testGlobalCommand in myScene.cpp
2) return either TCL_OK or TCL_ERROR depending on whether your command was successful or not
3) In the file myScene.cpp modify the function mySetScriptCommands and add a 
new Tcl_CreateCommand call. Look at the sample command "test" to learn how
to use it

Startup script
------------

every time you run you program the console will automatically execute the script start.tcl at
start up.  This script is located under the Build folder. For your projects you definitely 
want to modify the sample script that is provided.

Working directory
----------------
The working directory should be set to Build.

Custom scripts
------------

You can run your own scripts by using the shell command "source <script_path>"

Data folder
------------

The folder Build/data contains a few 3D models that you may use for your projects.

-----------------------------------------

============================================
MacOSX 10.6 and above
--------------------------------------------

- You need to run the program from the terminal. 
- Make sure your current working directory is Build.
- The event queues seem not to be handled properly. 
  You may need to press a button more than once to register
  an event. 



