import traci
import traci.constants as tc

import math
import random
import queue 

from bitarray import bitarray

sumoBinary = "C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo"
sumoCmd = [sumoBinary, "-c", "mahattan.sumocfg"]

traci.start(sumoCmd)

vehicles = {}	# create dictionary to hold key value of every vehicle

RANGE = 100

TASK_LOWER = 500
TASK_UPPER = 1000

FREQ_LOWER = 300
FREQ_UPPER = 500

CORES_LOWER = 1
CORES_UPPER = 4

class Network:

	def dist(self, src, dst):
		return math.sqrt( ((src.x - dst.x)**2) + ((src.y - dst.y)**2) )
		
	def delay(self, d):
	
		tslot = 50
		sigma = 200**2
		e = 0.001

		Ptx = 30
		theta = 5
		N0 = -174
		WMM = 75

		PL = 69.6 + 20.9*math.log10(d) + 0.001
		phi = Ptx - theta - N0*WMM - PL
		x = phi / (math.sqrt(2)*sigma)
		Phop = 1/2 *(1+math.erf(x))
		T = (2*tslot) / Phop

		return T

class Task:
	
	def __init__(self, mbits, src = 0, dst = 0, core = None):
		self.mbits = mbits
		self.src = src
		self.dst = dst
		self.core = core
		
class Processor:
	
	def __init__(self, cores = 1):
		self.cores = cores
		self.freq = random.randint(FREQ_LOWER,FREQ_UPPER) # homogeneous
		
		self.reset()

	def reset(self):
		self.blocked = bitarray(self.cores)
		self.blocked.setall(False)
		self.busy_counters = [0] * self.cores
		self.running_tasks = [None] * self.cores		
		
	def reset_core(self, core):
		self.blocked[core] = False
		self.busy_counters[core] = 0
		self.running_tasks[core] = None
			
	def provision(self):
		for core in range(self.cores):
			if self.blocked[core] == False:
				return core
		return None
	
	def allocate(self, core, task):
		exec_time = task.mbits/self.freq
		self.busy_counters[core] += exec_time
		self.blocked[core] = True
		self.running_tasks[core] = task
		return exec_time
	
	def tick(self):
		completed = [None] * self.cores	
		for core in range(self.cores):
			tmp = self.busy_counters[core]
			if self.blocked[core] == True:
				self.busy_counters[core] -= 1
				if self.busy_counters[core] <= 0:
					completed[core] = self.running_tasks[core]
					self.reset_core(core)					
		return completed

class Vehicle:
	global vehicles
	
	def	__init__(self, id, x, y):
		self.id = id		
		self.x = x
		self.y = y		
		
		self.neighbors = {}
		
		self.local_queue = queue.Queue(maxsize=9000)
		self.result_queue = queue.Queue(maxsize=1000)
		
		self.steps = 0 # time simulation
		
		self.proc = Processor(random.randint(CORES_LOWER,CORES_UPPER))
		self.netw = Network()
		
		self.reset()
		
	def reset(self):
		self.local = 0
		self.offloaded = 0
		self.generated = 0
		self.pending = 0
		self.received = 0
		self.delivered = 0
		
		self.delay = 0.0
		self.wait_time = 0.0
		self.total_exec_time = 0.0
		self.total_wait_time = 0.0
		self.total_delay = 0.0
		
		self.zero_hop_delivery = 0
		self.one_hop_delivery = 0
		self.two_hop_delivery = 0
		self.failures = 0
	
	def track(self, x, y):
		self.x = x
		self.y = y
		
	def compute_wait_time(self, task):
		self.wait_time += task.mbits/self.proc.freq
	
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
		self.add_local_queue(task)
		self.received += 1	
	
	def policy():
		if random.random() > 0.5 and len(self.neighbors) > 0:
			self.offload(task)
		else:
			self.add_local_queue(task)

		self.pending += 1
		
	def offload(self, task):
		dst = self.select()			
		task.dst = dst			
		obj = vehicles[dst]
		obj.receive(task)			
		vehicles[dst] = obj
		self.offloaded += 1
		self.total_delay += self.netw.delay(self.netw.dist(self,obj))
	
	def scan(self):
		for k,v in vehicles.items():
			d = math.sqrt( ((self.x - v.x)**2) + ((self.y - v.y)**2) )
			if d < RANGE and self.id != v.id:
				self.neighbors[v.id] = v
				
	def add_local_queue(self, task):		
		self.total_wait_time += self.wait_time
		self.local_queue.put(task)
		self.compute_wait_time(task)
	
	def generate(self):
		task = Task(random.randint(TASK_LOWER,TASK_UPPER), self.id)		
		self.generated += 1
		return task
					
	def delivery(self):
		while self.result_queue.empty() == False:
			task = self.result_queue.get()	# remove task from queue
			if task.src == self.id:
				self.local += 1
				self.pending -= 1
			else:
				obj = vehicles[task.src]
				d = self.netw.dist(self,obj)
				if d > 0 and d < RANGE:
					self.zero_hop_delivery += 1
				elif d >= RANGE and d < 2*RANGE:
					self.one_hop_delivery += 1
				elif d >= 2*RANGE and d <= 3*RANGE:
					self.two_hop_delivery += 1
				else:
					self.failures += 1
				obj.pending -= 1
				self.delivered += 1
				self.total_delay += self.netw.delay(d)

	def fetch(self):
		if self.local_queue.empty():
			return
		
		task = self.local_queue.queue[0] # peek		
		core = self.proc.provision()
		
		if core is not None:
			task = self.local_queue.get() # dequeue
			exec_time = self.proc.allocate(core,task)
			
			self.wait_time -= exec_time
			self.total_exec_time += exec_time
	
	def tick(self):		
		task = self.generate()
		
		self.scan()
		
		if random.random() > 0.5 and len(self.neighbors) > 0:
			self.offload(task)
		else:
			self.add_local_queue(task)

		self.pending += 1
			
		completed = self.proc.tick()	
		for task in completed:			
			if task is not None:
				self.result_queue.put(task)
		self.delivery()
		
		self.fetch() # new task
		self.steps = step
	
	def print_stats(self):
		print(f"vehicle id:{self.id}")		
		print(f"(generated, local, offloaded, received, pending, delivered):{self.generated},{self.local},{self.offloaded},{self.received},{self.pending},{self.delivered})")		
		print(f"(delay, wait time):({self.delay},{self.wait_time})")		
		print(f"(zero_hop,one_hop,two_hop,failures):({self.zero_hop_delivery},{self.one_hop_delivery},{self.two_hop_delivery},{self.failures})")
		
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
		obj.tick()
		vehicles[id] = obj
		
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
print ("  Execution time (s) = ", "{:.2f}".format(total_exec_time/total_executed))
print ("Network latency (ms) = ", "{:.2f}".format(total_delay/(total_offloaded+total_received)))
print (" Queuing latency (s) = ", "{:.2f}".format(total_wait_time/total_added))
print ("Throughput (mbits/s) = ", "{:.2f}".format(total_mbits/total_executed))
print ("        Failure rate = ", "{:.2f}".format(total_failures/total_offloaded))
print ("          Efficiency = ", "{:.2f}".format((total_local+(total_delivered-total_failures))/total_generated))
print ()
print ("*************************************************")
print ()

traci.close()