import random
import math
import queue

from constants import *

#import settings

import numpy as np
from processor import Processor
from network import Network
from profiler import Profiler
from experience import Experience

class Edge:
	
	def __init__(self, id, x, y):
		self.id = id		
		self.x = x
		self.y = y
		
		self.neighbors = []
		
		self.local_queue = queue.Queue(maxsize=MAX_QUEUE_LEN)
		self.result_queue = queue.Queue(maxsize=MAX_QUEUE_LEN)
		
		self.proc = Processor(random.randint(RSU_CORES_LOWER,RSU_CORES_UPPER))
		self.netw = Network()
		self.prof = Profiler()
		
		self.model = None
		self.experience = None
		self.n_episodes = 0
		self.loss = 0.0
		
		self.envstate = np.array([[0.0,0,0,0,0,0,0,0,0,0]])
		
		self.reset()
		
	def reset(self):
		self.steps = 0 # simulation time
		self.wait_time = 0.0
	
	def policy_random(self):
		return random.choice(self.neighbors)
		
	def policy_nearest(self,vehicles):
		mx = math.inf
		for i in range(len(self.neighbors)):
			obj = vehicles[self.neighbors[i]]
			d = self.netw.dist(self, obj)
			if d < mx:		
				mx = d
				selected = i
		return self.neighbors[selected]
		
	def policy_wait_time(self,vehicles):
		mx = math.inf
		for i in range(len(self.neighbors)):
			obj = vehicles[self.neighbors[i]]
			if obj.wait_time < mx:		
				mx = obj.wait_time
				selected = i
		return self.neighbors[selected]
		
	def select(self, vehicles):
		#return self.policy_random()
		#return self.policy_nearest(vehicles)
		return self.policy_wait_time(vehicles)		
	
	def receive(self, task):
		self.add(task)
		self.prof.pending += 1
		self.prof.received += 1
	
	def policy(self, vehicles):
		if random.random() > 0.5 and len(self.neighbors) > 0:
			self.offload(vehicles)
		else:
			self.fetch() # new task
		
	def offload(self, vehicles):
		if self.local_queue.empty():
			return
		task = self.local_queue.get() # dequeue
		dst = self.select(vehicles)
		task.dst = dst	
		obj = vehicles[dst]
		obj.receive(task)	
		vehicles[dst] = obj
		self.prof.pending -= 1
		self.prof.offloaded_to_fog += 1
		self.prof.total_delay += self.netw.delay(task,self.netw.latency(self.netw.dist(self,obj)))
	
	def scan(self, vehicles):
		self.neighbors.clear()
		for k,v in vehicles.items():
			d = math.sqrt( ((self.x - v.x)**2) + ((self.y - v.y)**2) )
			if d < RSU_RANGE and self.id != v.id:
				self.neighbors.append(v.id)
				
	def add(self, task):
		self.prof.added += 1
		self.prof.total_wait_time += self.wait_time
		self.local_queue.put(task)
		self.wait_time += task.mbits/self.proc.freq		
						
	def delivery(self, vehicles, active_vehicles):
		while self.result_queue.empty() == True: return

		task = self.result_queue.get()	# remove task from queue
		if task.src in active_vehicles:
			obj = vehicles[task.src]
			d = self.netw.dist(self,obj)
			if d <= RSU_RANGE:
				self.prof.zero_hop_delivery += 1
			elif d > RSU_RANGE and d <= 2*RSU_RANGE:
				self.prof.one_hop_delivery += 1
			elif d > 2*RSU_RANGE and d <= 3*RSU_RANGE:
				self.prof.two_hop_delivery += 1
			else:
				self.prof.failures += 1
			self.prof.delivered += 1
			self.prof.total_mbits += task.mbits
			self.prof.total_delay += self.netw.delay(task,self.netw.latency(d))
		else:
			self.prof.infinites += 1

	def fetch(self):
		if self.local_queue.empty():
			return
		
		task = self.local_queue.queue[0] # peek		
		core = self.proc.provision()
		
		if core is not None:
			task = self.local_queue.get() # dequeue
			exec_time = self.proc.allocate(core,task)
			
			self.wait_time -= exec_time
			self.prof.total_exec_time += exec_time
			self.prof.scheduled += 1
	
	def complete(self):
		completed = self.proc.tick()	
		for task in completed:			
			if task is not None:
				self.result_queue.put(task)
				self.prof.pending -= 1

	def tick(self, step,vehicles, active_vehicles):
		#print(step,":R:",len(vehicles))
		self.scan(vehicles)
		self.complete()
		self.delivery(vehicles, active_vehicles)
		self.policy(vehicles)
	
		self.steps = step
	
	def print_stats(self):
		print(f"edge id:{self.id}")		
		print(f"(local, offloaded, received, pending, delivered):{self.local},{self.offloaded},{self.received},{self.pending},{self.delivered})")		
		print(f"(wait time):({self.wait_time})")		
		print(f"(zero_hop,one_hop,two_hop,failures):({self.zero_hop_delivery},{self.one_hop_delivery},{self.two_hop_delivery},{self.failures})")