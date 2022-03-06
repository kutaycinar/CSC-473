# Spring Mass System

![devenv_AahS9gZtcR](https://user-images.githubusercontent.com/76612427/156911233-9be141bc-2dab-4ddd-92bf-5e6ab5a1aba8.gif)

In this assignment, your task is to develop a spring-mass system and a corresponding simulator that support an arbitrary number of particles. The particles can be connected with springs in arbitrary ways and are subject to gravity. There is a flat ground, oriented along the y-axis, that you should model particle collisions with using penalty forces. The simulator should support the following integration methods:  Forward Euler, Symplectic Euler, and Verlet.

You must implement the interface commands below. None of these commands need to work after the user has started the simulation. Additionally, during grading, these commands will only be called (during a particular invocation of the application) in the order in which they are given here (see instructions and clarifications, as well as, attachments).
1. `system <sys_name> dim <Number of Particles>`
This command initializes the particle system to hold up to the given number of particles. Particles may be initialized to the origin of the world, or may not appear until added by the next command.

1. `system <sys_name> particle <index> <mass> <x y z vx vy vz> `
This command sets a position, mass, and velocity for a given particle.

1. `system <sys_name> all_velocities  <vx vy vz> `
This command sets the velocity of all particles.

1. `simulator <sim_name> link <sys name> <Number of Springs>`
This links the simulator to a particular particle system and initializes it to work with a given number of springs.

1. `simulator <sim_name> spring <index1> <index2> <ks> <kd> <restlength>`
This sets up a given spring. If the rest length is a negative number, the system should automatically set the rest length of the spring to the distance between the corresponding particles at the time the command is given.

1. `simulator <sim_name> fix <index>`
This command nails particle <index> to its current position.

1. `simulator <sim_name> integration <euler|symplectic|verlet> <time step>`
This changes the integration technique used by the given simulator and sets the time step of the integration. 

1. `simulator <sim_name> ground <ks> <kd> `
Sets the parameters of the penalty forces applied to particles that try to go underground.

1. `simulator <sim_name> gravity <g>`
Sets the acceleration due to gravity, in unit length per unit time squared.

1. `simulator <sim_name> drag <kdrag>`
Sets the global drag (friction) coefficient (Fi = -kdrag vi).  The command expects a positive number 
