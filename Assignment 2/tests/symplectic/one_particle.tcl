system partSys dim 1
system partSys particle 0 1 0 1 0 0 0 0 
simulator partSim link partSys 1
#simulator partSim integration euler .01
simulator partSim integration symplectic .01
#simulator partSim integration verlet .01
simulator partSim ground 1000 20
simulator partSim gravity -9.8





