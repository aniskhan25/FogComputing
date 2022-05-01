import random
import math
import queue

from constants import *
from globals import *

from processor import Processor
from network import Network
from profiler import Profiler

class Edge:
	global vehicles
	global edges
	
	def	__init__(self, id, x, y):
		self.id = id		
		self.x = x
		self.y = y
		
		self.neighbors = {}
		
		self.local_queue = queue.Queue(maxsize=MAX_QUEUE_LEN)
		self.result_queue = queue.Queue(maxsize=MAX_QUEUE_LEN)
		
		self.proc = Processor(random.randint(RSU_CORES_LOWER,RSU_CORES_UPPER))
		self.netw = Network()
		self.prof = Profiler()
		
		self.reset()
		
	def reset(self):
		self.steps = 0 # simulation time
		self.wait_time = 0.0
	
	def policy_random(self):
		return random.choice(list(self.neighbors.keys()))		
		
	def policy_nearest(self):
		mx = math.inf
		for k,v in self.neighbors.items():
			d = self.netw.dist(self,v)		
			if d < mx:				
				mx = d
				selected = k
		return selected
		
	def policy_wait_time(self):
		mx = math.inf
		for k,v in self.neighbors.items():
			if v.wait_time < mx:				
				mx = v.wait_time
				selected = k
		return selected
		
	def select(self):
		return self.policy_random()
		#return self.policy_nearest()
		#return self.policy_wait_time()		
	
	def receive(self, task):
		self.add(task)
		self.prof.received += 1	
	
	def policy():
		if random.random() > 0.5 and len(self.neighbors) > 0:
			self.offload(task)
		else:
			self.add(task)

		self.prof.pending += 1
		
	def offload(self):
		if self.local_queue.empty():
			return
		task = self.local_queue.get() # dequeue
		dst = self.select()
		task.dst = dst	
		obj = vehicles[dst]
		obj.receive(task)		
		vehicles[dst] = obj
		self.prof.offloaded_to_fog += 1
		self.prof.total_delay += self.netw.delay(self.netw.dist(self,obj))
	
	def scan(self):
		for k,v in vehicles.items():
			d = math.sqrt( ((self.x - v.x)**2) + ((self.y - v.y)**2) )
			if d < RSU_RANGE and self.id != v.id:
				self.neighbors[v.id] = v
				
	def add(self, task):
		self.prof.added += 1
		self.prof.total_wait_time += self.wait_time
		self.local_queue.put(task)
		self.wait_time += task.mbits/self.proc.freq		
						
	def delivery(self):
		while self.result_queue.empty() == False:
			task = self.result_queue.get()	# remove task from queue			
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
			obj.prof.pending -= 1
			self.prof.delivered += 1
			self.prof.total_mbits += task.mbits
			self.prof.total_delay += self.netw.delay(d)

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
			self.prof.executed += 1
	
	def tick(self, step):
				
		self.scan()
		
		completed = self.proc.tick()	
		for task in completed:			
			if task is not None:
				self.result_queue.put(task)
		self.delivery()
		
		if random.random() > 0.5 and len(self.neighbors) > 0:
			self.offload()
		else:
			self.fetch() # new task
		
		self.steps = step
	
	def print_stats(self):
		print(f"edge id:{self.id}")		
		print(f"(local, offloaded, received, pending, delivered):{self.local},{self.offloaded},{self.received},{self.pending},{self.delivered})")		
		print(f"(wait time):({self.wait_time})")		
		print(f"(zero_hop,one_hop,two_hop,failures):({self.zero_hop_delivery},{self.one_hop_delivery},{self.two_hop_delivery},{self.failures})")