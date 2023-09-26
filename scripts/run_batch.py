#!/usr/bin/env python

import subprocess

nruns = 15
#  run_duration = 1205 # seconds
#  plannerTypes = ["rrt nRobots:=1", "rrt nRobots:=2", "rrt nRobots:=3", "rrt nRobots:=4"]
run_duration = 605 # seconds
plannerTypes = ["dmcts", "cluster", "mcts"]
cmd = f"timeout {run_duration}s roslaunch dmce_sim demo.launch restrictComms:=false scenario:=urban plannerType:="

cleanerProcess = subprocess.Popen("make logclean build-pkg", shell=True)
cleanerProcess.wait()

for plannerType in plannerTypes:
	finalCmd = cmd + plannerType
	for i in range(nruns):
		print(f"\n\n##########\n\nStarting run {i}\nCommand: {finalCmd}\n\n##########\n\n")
		process = subprocess.Popen(finalCmd, shell=True)
		process.wait()
