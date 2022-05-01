import traci
import traci.constants as tc

import random

from constants import *
from globals import *

from profiler import Profiler
from edge import Edge
from vehicle-two-tier import Vehicle

sumoBinary = "C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo"
sumoCmd = [sumoBinary, "-c", "mahattan.sumocfg"]

traci.start(sumoCmd)

# id, xloc, yloc
edge1 = Edge(1, 1711.03, 2341.07)
edge2 = Edge(2, 2257.12, 2273.51)
edge3 = Edge(3, 1656.19, 2046.40)
edge4 = Edge(4, 1974.48, 2363.03)

edges[edge1.id] = edge1
edges[edge2.id] = edge2
edges[edge3.id] = edge3
edges[edge4.id] = edge4

n_steps = 600
step = 0
#random.seed(1)
while step < n_steps:
	#print("Step : ", step)
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
	
	for k,v in edges.items():	
		v.tick(step)
		
	step += 1
	
total_generated = 0
total_added = 0
total_local = 0
total_offloaded = 0
total_pending = 0
total_received = 0
total_delivered = 0
total_executed = 0

total_offloaded_to_fog = 0
total_delivered_to_edge = 0

total_mbits = 0.0
total_delay = 0.0
total_wait_time = 0.0
total_exec_time = 0.0
total_zero_hop_delivery = 0
total_one_hop_delivery = 0
total_two_hop_delivery = 0
total_failures = 0

for k,v in vehicles.items():
	total_generated += v.prof.generated
	total_added += v.prof.added
	total_local += v.prof.local
	total_offloaded += v.prof.offloaded
	total_pending += v.prof.pending
	total_received += v.prof.received
	total_delivered += v.prof.delivered
	total_executed += v.prof.executed

	total_offloaded_to_fog += v.prof.offloaded_to_fog
	total_delivered_to_edge += v.prof.delivered_to_edge
	
	total_mbits += v.prof.total_mbits
	total_delay += v.prof.total_delay	
	total_wait_time += v.prof.total_wait_time
	total_exec_time += v.prof.total_exec_time
	total_zero_hop_delivery += v.prof.zero_hop_delivery
	total_one_hop_delivery += v.prof.one_hop_delivery
	total_two_hop_delivery += v.prof.two_hop_delivery
	total_failures += v.prof.failures

for k,v in edges.items():
	total_added += v.prof.added
	total_local += v.prof.local
	total_offloaded += v.prof.offloaded	
	total_pending += v.prof.pending
	total_received += v.prof.received
	total_delivered += v.prof.delivered
	total_executed += v.prof.executed
	
	total_offloaded_to_fog += v.prof.offloaded_to_fog
	total_delivered_to_edge += v.prof.delivered_to_edge
	
	total_mbits += v.prof.total_mbits
	total_delay += v.prof.total_delay
	total_wait_time += v.prof.total_wait_time
	total_exec_time += v.prof.total_exec_time
	total_zero_hop_delivery += v.prof.zero_hop_delivery
	total_one_hop_delivery += v.prof.one_hop_delivery
	total_two_hop_delivery += v.prof.two_hop_delivery
	total_failures += v.prof.failures

print ()
print ()
print ("******************** STATS **********************")
print ()
print ("            Vehicles entering simulation = ", len(vehicles))
print ("                 Tasks generated at user = ", total_generated)
print ("  Tasks added queued (user + edge + fog) = ", total_added)
print ("                 Tasks completed at user = ", total_local)
print ("               Tasks offloaded from user = ", total_offloaded)
print ("    Tasks pending at (user + edge + fog) = ", total_pending)
print (" Tasks received at fog-tier (edge + fog) = ", total_received)
print ("Tasks delivered by fog-tier (edge + fog) = ", total_delivered)
print ("   Tasks executed at (user + edge + fog) = ", total_executed)
print ()
print ("Tasks offloaded (edge -> fog) = ", total_offloaded_to_fog)
print ("Total delivered (fog -> edge) = ", total_delivered_to_edge)
print ()
print ("****************** HANDOVER *********************")
print ()
print ("Zero hop delivery = ", total_zero_hop_delivery)
print (" One hop delivery = ", total_one_hop_delivery)
print (" Two hop delivery = ", total_two_hop_delivery)
print ("   Total failures = ", total_failures)
print ()
print ("***************** PERF METRICS ******************")
print ()
print ("  Execution time (s) = ", "{:.3f}".format(total_exec_time/total_executed))
print ("Network latency (ms) = ", "{:.3f}".format(total_delay/(total_offloaded+total_received)))
print (" Queuing latency (s) = ", "{:.3f}".format(total_wait_time/total_added))
print ("Throughput (mbits/s) = ", "{:.3f}".format(total_mbits/total_executed))
print ("        Failure rate = ", "{:.3f}".format(total_failures/total_offloaded))
print ("          Efficiency = ", "{:.3f}".format((total_local+(total_delivered-total_failures))/total_generated))
print ()
print ("*************************************************")
print ()

traci.close()