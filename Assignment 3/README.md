# Inverse Kinematics

![devenv_DYYN5u8YBC](https://user-images.githubusercontent.com/76612427/156911823-b4b28480-3e25-4177-aab2-9a7691f8b6c3.gif)

In this assignment, your task is to develop an Inverse Kinematics engine for animating a human character. The human character is to be positioned in front of a blackboard. Your program should read an arbitrary shape represented by a 2D spline (use code from assignment 1 or the spline class provided). The character should be able to draw (i.e trace) the spline on the blackboard.

The program should work as follows. Through the scripting, the commands read an arbitrary spline. Once the spline is read, pressing the key “s” should make the character draw the spline repeatedly. The drawing should stop when “s” is pressed again. Please read the entirety of this assignment text as it includes important requirements and several notes at the end. There is also a short lecture associated with this assignment to help guide you. which we will cover in class.

## Tasks

1. Model a classroom using one wall whose visible surface is the z = 0 plane, (where the blackboard will be attached), and a planar floor with the y-axis as its normal.
1. Model and position a blackboard. It should be planar and in the y-x plane. When the blackboard is positioned on the wall, its geometric center should be at the origin of the world coordinate system, <0, 0, 0>.
1. You should be able to read arbitrary splines and project them on the blackboard.
1. Model a human character using ellipses (see figure).
1. Implement an inverse kinematics solver using the pseudo-inverse approach. If you get stuck here/run out of time/can not complete the solver for whatever reason, you may use the Jacobian Transpose or the Cyclic Coordinate Descent method for a maximum of 5 points. You must identify the approach you took in the included readme file.
1. Draw the spline on the board.
1. The character should start in the rest pose. After a spline is loaded, the character should switch to the Ready pose (see below) for drawing on the blackboard.
1. Follow the interface elements specified below.
