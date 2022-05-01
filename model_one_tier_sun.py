import traci
import traci.constants as tc
import random

from sys import platform

from constants import *
from globals import *

from profiler import Profiler
from vehicle_one_tier_sun import Vehicle

if platform == "linux" or platform == "linux2":
	sumoBinary = "/usr/bin/sumo"
elif platform == "darwin":
	# OS X
	# print("ERR")
	sumoBinary = "/Users/anisrahman/sumo/bin/sumo"
elif platform == "win32":
	sumoBinary = "C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo"

sumoCmd = [sumoBinary, "-c", "grid.sumocfg", "--no-internal-links", "--step-length", str(SIM_STEP_SIZE)]
#sumoCmd = [sumoBinary, "-c", "grid.sumocfg", "--no-internal-links"]

traci.start(sumoCmd)
n_steps = 1000
		
step = 1
#random.seed(1)
while step < n_steps:	
	traci.simulationStep()
	
	for id in traci.vehicle.getIDList():
		pos = traci.vehicle.getPosition(id)
		obj = None		
		if id in vehicles:
			obj = vehicles[id]
			obj.track(pos[0], pos[1])
		else:
			obj = Vehicle(id, pos[0], pos[1])		
		obj.tick(step)
		vehicles[id] = obj
		active_vehicles[id] = obj
	
	for i in list(active_vehicles):
		if active_vehicles[i].steps != step:
			del active_vehicles[i]
			
	for k in list(seen):
			if k not in active_vehicles:
				del seen[k]
				
	step += SIM_STEP_SIZE

printstats()

traci.close()
