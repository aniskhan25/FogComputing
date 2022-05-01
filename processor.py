import random

from bitarray import bitarray

from constants import *

class Processor:
	
	def __init__(self, cores = 1):
		self.cores = cores
		self.freq = random.uniform(FREQ_LOWER,FREQ_UPPER) # homogeneous
		
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
				self.busy_counters[core] -= SIM_STEP_SIZE
				if self.busy_counters[core] <= 0:
					completed[core] = self.running_tasks[core]
					self.reset_core(core)					
		return completed
