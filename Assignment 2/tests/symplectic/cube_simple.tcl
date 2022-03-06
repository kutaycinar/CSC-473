set springks 200
set springkd 20

system partSys dim 8
system partSys particle 0 10 1 1 1 0 0 0
system partSys particle 1 10 1 8 1 0 0 0
system partSys particle 2 10 8 8 1 0 0 0
system partSys particle 3 10 8 1 1 0 0 0
system partSys particle 4 10 1 1 8 0 0 0
system partSys particle 5 10 1 8 8 0 0 0
system partSys particle 6 10 8 8 8 0 0 0
system partSys particle 7 10 8 1 8 0 0 0

 simulator partSim link partSys 28
set strength 7
simulator partSim spring 0 1 $springks $springkd $strength
simulator partSim spring 1 2 $springks $springkd $strength
simulator partSim spring 2 3 $springks $springkd $strength
simulator partSim spring 0 3 $springks $springkd $strength

simulator partSim spring 5 6 $springks $springkd $strength
simulator partSim spring 6 7 $springks $springkd $strength
simulator partSim spring 7 4 $springks $springkd $strength
simulator partSim spring 4 5 $springks $springkd $strength

simulator partSim spring 1 5 $springks $springkd $strength
simulator partSim spring 6 2 $springks $springkd $strength
simulator partSim spring 7 3 $springks $springkd $strength
simulator partSim spring 0 4 $springks $springkd $strength

# simulator partSim spring 6 3 $springks $springkd 7.41
# simulator partSim spring 7 2 $springks $springkd 7.41
# simulator partSim spring 3 1 $springks $springkd 7.41
# simulator partSim spring 2 0 $springks $springkd 7.41
#
# simulator partSim spring 5 2 $springks $springkd 7.41
# simulator partSim spring 6 1 $springks $springkd 7.41
# simulator partSim spring 4 1 $springks $springkd 7.41
# simulator partSim spring 5 0 $springks $springkd 7.41
#
# simulator partSim spring 0 7 $springks $springkd 7.41
# simulator partSim spring 3 4 $springks $springkd 7.41
# simulator partSim spring 2 5 $springks $springkd 7.41
# simulator partSim spring 1 6 $springks $springkd 7.41
#
# simulator partSim spring 2 4 $springks $springkd 7.41
# simulator partSim spring 1 7 $springks $springkd 7.41
# simulator partSim spring 3 5 $springks $springkd 7.41
# simulator partSim spring 0 6 $springks $springkd 7.41

simulator partSim integration symplectic .01
simulator partSim ground 5000 300
simulator partSim gravity -10.8
