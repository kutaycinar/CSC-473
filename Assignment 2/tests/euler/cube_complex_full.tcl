
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
simulator partSim spring 1 0 $springks $springkd 7.0
simulator partSim spring 2 0 $springks $springkd 9.8995
simulator partSim spring 2 1 $springks $springkd 7.0
simulator partSim spring 3 0 $springks $springkd 7.0
simulator partSim spring 3 1 $springks $springkd 9.8995
simulator partSim spring 3 2 $springks $springkd 7.0
simulator partSim spring 4 0 $springks $springkd 7.0
simulator partSim spring 4 1 $springks $springkd 9.8995
simulator partSim spring 4 2 $springks $springkd 12.1244
simulator partSim spring 4 3 $springks $springkd 9.8995
simulator partSim spring 5 0 $springks $springkd 9.8995
simulator partSim spring 5 1 $springks $springkd 7.0
simulator partSim spring 5 2 $springks $springkd 9.8995
simulator partSim spring 5 3 $springks $springkd 12.1244
simulator partSim spring 5 4 $springks $springkd 7.0
simulator partSim spring 6 0 $springks $springkd 12.1244
simulator partSim spring 6 1 $springks $springkd 9.8995
simulator partSim spring 6 2 $springks $springkd 7.0
simulator partSim spring 6 3 $springks $springkd 9.8995
simulator partSim spring 6 4 $springks $springkd 9.8995
simulator partSim spring 6 5 $springks $springkd 7.0
simulator partSim spring 7 0 $springks $springkd 9.8995
simulator partSim spring 7 1 $springks $springkd 12.1244
simulator partSim spring 7 2 $springks $springkd 9.8995
simulator partSim spring 7 3 $springks $springkd 7.0
simulator partSim spring 7 4 $springks $springkd 7.0
simulator partSim spring 7 5 $springks $springkd 9.8995
simulator partSim spring 7 6 $springks $springkd 7.0

simulator partSim integration euler .001

simulator partSim ground 5000 300
simulator partSim gravity -10.0
