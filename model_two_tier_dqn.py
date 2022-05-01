# https://www.samyzaf.com/ML/rl/qmaze.html

from __future__ import print_function

import os, sys, time, datetime, json, random, math, queue

from sys import platform

import traci
import traci.constants as tc

#import settings

from constants import *
from constants_dqn import *

from profiler import Profiler
from vehicle_two_tier_dqn import Vehicle
from edge_dqn import Edge
from experience import Experience

import numpy as np
from keras.models import Sequential
from keras.layers.core import Dense, Activation
from keras.optimizers import SGD , Adam, RMSprop
from keras.layers.advanced_activations import PReLU
import matplotlib.pyplot as plt

if platform == 'linux' or platform == 'linux2':
    sumoBinary = '/usr/bin/sumo'
elif platform == 'darwin':
    sumoBinary = '/Users/anisrahman/sumo/bin/sumo'
elif platform == 'win32':
    sumoBinary = 'C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo'

sumoCmd = [sumoBinary, '-c', 'manhattan.sumocfg', '--no-internal-links', '--step-length', str(SIM_STEP_SIZE)]
#sumoCmd = [sumoBinary, "-c", "grid.sumocfg", "--no-internal-links", "--step-length", str(SIM_STEP_SIZE)]
#sumoCmd = [sumoBinary, "-c", "grid.sumocfg", "--no-internal-links"]

#global edges
#global vehicles

#settings.init()

edges = {}
vehicles = {}
active_vehicles = {}
seen = {}

def qtrain(**opt):
	
	n_epoch = opt.get('n_epoch', 15000)
	max_memory = opt.get('max_memory', 5000)
	data_size = opt.get('data_size', 50)	
	
	start_time = datetime.datetime.now()
	
	# Initialise experience replay object
	for k,v in edges.items():
		v.experience = Experience(v.model, max_memory=max_memory)
		
	for epoch in range(n_epoch):
	
		traci.start(sumoCmd)
						
		vehicles.clear()
	
		for k,v in edges.items():	
			v.n_episodes = 0
			
		n_steps = 1000
		step = 0
		#random.seed(1)
		while step < n_steps:
			print("Step : ", step, end='\r')
			#print(step,":M:",len(vehicles))
			traci.simulationStep()

			for id in traci.vehicle.getIDList():
				pos = traci.vehicle.getPosition(id)
				obj = None
				if id in vehicles:
					obj = vehicles[id]
					obj.track(pos[0], pos[1])
				else:
					obj = Vehicle(id, pos[0], pos[1])
					
					# get initial envstate (1d flattened canvas)
					#envstate = obj.observe()
		
				obj.tick(step, vehicles, edges, active_vehicles)
				vehicles[id] = obj
				active_vehicles[id] = obj
				
			for i in list(active_vehicles):
				if active_vehicles[i].steps != step:
					del active_vehicles[i]
			
			for k in list(seen):
				if k not in active_vehicles:
					del seen[k]

			for k,v in edges.items():
				v.tick(step, vehicles, active_vehicles)

			step += SIM_STEP_SIZE

		printstats()
			
		traci.close()
			
		total_generated = 0
		total_local = 0
		total_delivered = 0
		total_failures = 0

		for k,v in vehicles.items():
			total_generated += v.prof.generated
			total_local += v.prof.local	
			total_delivered += v.prof.delivered	
			total_failures += v.prof.failures

		loss = 0.0
		n_episodes = 0
		for k,v in edges.items():
			loss += v.loss
			n_episodes += v.n_episodes
			
		loss /= len(edges)
		n_episodes /= len(edges)
		
		if total_generated == 0:
			throughput = 1.0
		else:
			throughput = (total_local+(total_delivered-total_failures))/total_generated

		dt = datetime.datetime.now() - start_time
		t = format_time(dt.total_seconds())
		template = "Epoch: {:03d}/{:d} | Loss: {:.4f} | Episodes: {:f} | Throughput: {:.4f} | time: {}"
		print(template.format(epoch, n_epoch-1, loss, n_episodes, throughput, t))
		# we simply check if training has exhausted all free cells and if in all
		# cases the agent won
		#if throughput > 0.9 : epsilon = 0.05
		#if sum(win_history[-hsize:]) == hsize and completion_check(model, qmaze):
		#	print("Reached 100%% win rate at epoch: %d" % (epoch,))
		#	break

	end_time = datetime.datetime.now()
	dt = datetime.datetime.now() - start_time
	seconds = dt.total_seconds()
	t = format_time(seconds)
	print("n_epoch: %d, max_mem: %d, data: %d, time: %s" % (epoch, max_memory, data_size, t))
	return seconds

# self.is a small utility for printing readable time strings:
def format_time(seconds):
	if seconds < 400:
		s = float(seconds)
		return "%.1f seconds" % (s,)
	elif seconds < 4000:
		m = seconds / 60.0
		return "%.2f minutes" % (m,)
	else:
		h = seconds / 3600.0
		return "%.2f hours" % (h,)
		
		
def build_model(size, lr=0.001):
	model = Sequential()
	model.add(Dense(size, input_shape=(size,)))
	model.add(PReLU())
	model.add(Dense(size))
	model.add(PReLU())
	model.add(Dense(num_actions))
	model.compile(optimizer='adam', loss='mse')
	return model

def printstats():
	global vehicles

	total_generated = 0
	total_added = 0
	total_local = 0
	total_offloaded = 0
	total_pending = 0
	total_received = 0
	total_delivered = 0
	total_scheduled = 0

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
	total_infinites = 0

	total_explored = 0
	total_exploited = 0

	total_neighbors = 0

	print(len(vehicles))

	for k,v in vehicles.items():
		total_generated += v.prof.generated
		total_added += v.prof.added
		total_local += v.prof.local
		total_offloaded += v.prof.offloaded
		total_pending += v.prof.pending
		total_received += v.prof.received
		total_scheduled += v.prof.scheduled

		total_delivered_to_edge += v.prof.delivered_to_edge
	
		total_mbits += v.prof.total_mbits
		total_delay += v.prof.total_delay	
		total_wait_time += v.prof.total_wait_time
		total_exec_time += v.prof.total_exec_time
		total_failures += v.prof.failures
		total_infinites += v.prof.infinites
	
		total_explored += v.explored
		total_exploited += v.exploited
	
		total_neighbors += v.prof.neighbors

	for k,v in edges.items():
		total_zero_hop_delivery += v.prof.zero_hop_delivery
		total_one_hop_delivery += v.prof.one_hop_delivery
		total_two_hop_delivery += v.prof.two_hop_delivery

		total_delivered += v.prof.delivered
		total_infinites += v.prof.infinites

		total_offloaded_to_fog += v.prof.offloaded_to_fog

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
	print ("  Tasks scheduled at (user + edge + fog) = ", total_scheduled)
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
	print ("  Total infinites = ", total_infinites)
	print ()
	print ("***************** PERF METRICS ******************")
	print ()
	if total_scheduled != 0: print ("  Execution time (s) = ", "{:.3f}".format(total_exec_time/total_scheduled))
	if total_offloaded != 0 & total_received !=0: print ("Network latency (ms) = ", "{:.3f}".format(total_delay/(total_offloaded+total_received)))
	if total_added != 0: print (" Queuing latency (s) = ", "{:.3f}".format(total_wait_time/total_added))
	if total_scheduled != 0: print ("Throughput (mbits/s) = ", "{:.3f}".format(total_mbits/total_scheduled))
	if total_offloaded != 0: print ("        Failure rate = ", "{:.3f}".format(total_failures/total_offloaded))
	if total_generated != 0: print ("          Efficiency = ", "{:.3f}".format((total_local+(total_delivered-total_failures-total_infinites))/total_generated))
	print ()
	print ("*************************************************")
	print ()

	print ("Explored = ", total_explored)
	print ("Exploited = ", total_exploited)
	if total_offloaded != 0: print ("Neighbors = ", total_neighbors / total_offloaded)

size = 10

def deploy_edges():
	id = 0
	for x in range(100,901,400):
		for y in range(100,901,400):
			edges[id] = Edge(id, x, y)
			edges[id].model = build_model(size)
			id += 1

deploy_edges()

print(edges)

qtrain(n_epoch=1, max_memory=8*size, data_size=32)
#qtrain(n_epoch=5, max_memory=8*size, data_size=32, weights_file='model_210203.h5', name='model_210203')
