# Hermite Spline Modelling and Animation

![devenv_eTyJ29jvNw](https://user-images.githubusercontent.com/76612427/156910496-8fa5b30b-c905-4768-bf83-a23187a2f9d0.gif)

## Part 1: Hermite Spline modelling

For this question, you have to create a Hermite spline class.
Note: Names enclosed in < and >, indicate values you need to provide (without providing the “<” and “>”). Name variables are text strings, anything indicated shown as an index is an integer, and all other numbers are floating point values.

[3] Hermite spline class with a maximum of 40 control points. 
[3] Catmull-Rom initialization with second-order accurate boundary conditions using the following command:
    > system <name> cr 

[3] Manipulation of control points and tangents using the console:
    > system <name> set tangent  <index> <x y z>
    > system <name> set point <index> <x y z>
    > system <name> add point <x y z sx sy sz>  # adds a point with the given tangent at the end of the spline.

[5] Arc Length parameterization using a piecewise linear approximation and lookup table. The following function call should print the arc length of a spline up to parametric value t. Parameter t, in this case, goes from 0 to 1 for the entire spline, that is for t =1 you get the length of the entire spline.
    > system <name> getArcLength <t>

[3] Input/output  from/to  a file. 
    > system <name> load "<file name>"
    > system <name> export "<file name>"

    Implement exactly the following format for the spline file:
    <Spline Name> <n> 
    <p1[x] p1[y] p1[z] s1[x] s1[y] s1[z]>
    <p2[x] p2[y] p2[z] s2[x] s2[y] s2[z]>
        ….
    <pn[x] pn[y] pn[z] pn[x] sn[y] sn[z]>
    where the prefix “p” refers to control points and the prefix “s” to tangents.
    When reading a file you may ignore the name of the spline stored in it. The template code does not allow the renaming of objects.

[1] Spline drawing. Remember we are using old open GL, see ref here. There are also tips in the course forum. In the base code, util/GLutilities.cxx has helper functions and pointers on how to use old OpenGL for drawing.
[2] Quality. (Reaching the target, rendering, coding style, clearing the spline when the user hits ‘r’.)


## Part 2: Animate a scene using Hermite splines
      
For this part, you have to animate a moving target object on a plane surface and another, intersecting, object that hits the target object (Internally, these are a 'tank' and a 'missile', respectively, from a gaming example, but you may use any of the provided object models--*.obj). The target (tank) should follow any path you load for it. You should automatically calculate a trajectory for the intersecting object (missile) as a two-point Hermite spline.

[1] Loading the path for the target: “> system tankpath load "<spline file>"
[2] The intersecting object (missile) should start at an arbitrary orientation and position.  Implement the following syntax exactly:
    > system missile  state  <x y z v1 v2 v3 e> 
where x y z is the position and [v1 v2 v3 e] the orientation quaternion (scalar part in the end).

[4] You need to align the major axis of the intersecting object (missile) with the tangent of the path. Twist along the major axis can be ignored. Use the quaternion functionality of glm, or any library you prefer.
[2] The moving target object (tank) (ONLY) should start and stop in an easy-in easy-out way (10% of the length of the curve for each, i.e., 10% at the beginning 10% at the end) and should have constant speed during the rest of the motion.  Print the speed of the car on the console once per second.
[2] Compute the trajectory for the intersecting object (missile). The intersecting object (missile) should hit the target (tank) from behind i.e. you need to align the end tangent of the missile’s path with the current tangent of the target’s motion.
[2] The intersecting object (missile) is moving with a linear speed of constant magnitude. You should be able to set this speed in meters per second as follows: > system missile speed <value>
[1] When the user presses ‘r’ the target and intersecting object should assume the position and velocity they had when the simulation started.
[1] Quality. Make sure that the scale of your objects is appropriate for the tests provided.

## Part 3: Command Line Arguments
      
Add the following commands to the interpreter in addition to the ones specified in the assignment:

[1] Add the command “part1” to the console. When I type “part1” on the console you should instantiate a hermite-spline system called “hermite”. At this point, I should be able to load a spline and see it on screen. Also, I should be able to refer to your system using the name “hermite”. You can do this by using the SetName method.
        Example: system hermite load "mySplinePath.txt"
        should load the spline file “mySplinePath.txt”
        Note: In this case ignore the name of the spline stored in the file.

[1] Add the command “part2” to the console. When I type “part2” on the console you should instantiate all objects related to the second part of the project. Most importantly you should have two items named “tankpath” and “missile” that behave as described in the main specs for the assignment.
