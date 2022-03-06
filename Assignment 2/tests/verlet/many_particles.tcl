set num 1000
system partSys dim $num

set count 0
while {$count < $num} {
	system partSys particle $count 1 [expr $count * rand() * rand()] [expr $count * rand() * rand()] [expr $count * rand() * rand()] [expr $count * rand() * rand()] [expr $count * rand() * rand()] [expr $count * rand() * rand()] 
	set count [expr $count + 1]
}
simulator partSim link partSys $num
simulator partSim integration verlet .001
#simulator partSim integration symplectic .01
#simulator partSim integration euler .001
simulator partSim ground 1000 200
simulator partSim gravity -9.8






