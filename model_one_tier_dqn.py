from __future__ import print_function

import os, sys, time, datetime, json, random, math, queue

import traci
import traci.constants as tc

from constants import *
from constants_dqn import *
from globals import *

from profiler import Profiler
from vehicle_dqn import Vehicle
from experience import Experience

import numpy as np
from keras.models import Sequential
from keras.layers.core import Dense, Activation
from keras.optimizers import SGD , Adam, RMSprop
from keras.layers.advanced_activations import PReLU
import matplotlib.pyplot as plt

sumoBinary = "C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo"
sumoCmd = [sumoBinary, "-c", "mahattan.sumocfg"]

def qtrain(model, **opt):
	global vehicles
	global epsilon
	
	n_epoch = opt.get('n_epoch', 15000)
	max_memory = opt.get('max_memory', 1000)
	data_size = opt.get('data_size', 50)
	weights_file = opt.get('weights_file', "")
	name = opt.get('name', 'model')
	start_time = datetime.datetime.now()

	print(n_epoch,":",max_memory)
	# If you want to continue training from a previous model,
	# just supply the h5 file name to weights_file option
	if weights_file:
		print("loading weights from file: %s" % (weights_file,))
		model.load_weights(weights_file)

	# Construct environment/game from numpy array: maze (see above)
	#qmaze = Qmaze(maze)

	# Initialize experience replay object
	experience = Experience(model, max_memory=max_memory)

	#win_history = []	# history of win/lose game
	#n_free_cells = len(qmaze.free_cells)
	#hsize = qmaze.maze.size//2	# history window size
	#win_rate = 0.0
	#imctr = 1

	for epoch in range(n_epoch):
	
		traci.start(sumoCmd)
		
		loss = 0.0
		
		vehicles = {}
		
		n_episodes = 0
		n_steps = 6
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
					
					valid_actions = obj.valid_actions()
					if not valid_actions: break
					prev_envstate = envstate
					
					task = obj.generate()
					
					obj.scan()
		
					# Get next action
					if np.random.rand() < epsilon:
						action = random.choice(valid_actions)
					else:
						action = np.argmax(experience.predict(prev_envstate))

					# Apply action, get reward and new envstate
					envstate, reward, efficiency = obj.act(action,task)
					
					# Store episode (experience)
					episode = [prev_envstate, action, reward, envstate, efficiency]
					experience.remember(episode)
					n_episodes += 1
		
					# Train neural network model
					inputs, targets = experience.get_data(data_size=data_size)
					h = model.fit(
						inputs,
						targets,
						epochs=8,
						batch_size=16,
						verbose=0,
					)
					loss = model.evaluate(inputs, targets, verbose=0)
					
				else:
					obj = Vehicle(id, pos[0], pos[1])
					
					# get initial envstate (1d flattened canvas)
					envstate = obj.observe()					
		
				obj.tick(step)
				vehicles[id] = obj
				
			step += 1
			
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

		if total_generated == 0:
			throughput = 1.0
		else:
			throughput = (total_local+(total_delivered-total_failures))/total_generated

		dt = datetime.datetime.now() - start_time
		t = format_time(dt.total_seconds())
		template = "Epoch: {:03d}/{:d} | Loss: {:.4f} | Episodes: {:d} | Throughput: {:.4f} | time: {}"
		print(template.format(epoch, n_epoch-1, loss, n_episodes, throughput, t))
		# we simply check if training has exhausted all free cells and if in all
		# cases the agent won
		#if throughput > 0.9 : epsilon = 0.05
		#if sum(win_history[-hsize:]) == hsize and completion_check(model, qmaze):
		#	print("Reached 100%% win rate at epoch: %d" % (epoch,))
		#	break

	# Save trained model weights and architecture, self.will be used by the visualization code
	h5file = name + ".h5"
	json_file = name + ".json"
	model.save_weights(h5file, overwrite=True)
	with open(json_file, "w") as outfile:
		json.dump(model.to_json(), outfile)
	end_time = datetime.datetime.now()
	dt = datetime.datetime.now() - start_time
	seconds = dt.total_seconds()
	t = format_time(seconds)
	print('files: %s, %s' % (h5file, json_file))
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

size = 10
model = build_model(size)
qtrain(model, n_epoch=1, max_memory=8*size, data_size=32)
#qtrain(model, n_epoch=5, max_memory=8*size, data_size=32, weights_file='model_200517.h5', 'name', 'model_200517')