class Profiler:
	
	def	__init__(self):
		self.reset()
	
	def reset(self):
		self.generated = 0
		self.added = 0
		self.local = 0		
		self.offloaded = 0		
		self.pending = 0
		self.received = 0
		self.delivered = 0
		self.scheduled = 0
		
		self.offloaded_to_fog = 0
		self.delivered_to_edge = 0
		
		self.total_mbits = 0.0
		self.total_exec_time = 0.0
		self.total_wait_time = 0.0
		self.total_delay = 0.0
		self.total_reward = 0.0
		
		self.zero_hop_delivery = 0
		self.one_hop_delivery = 0
		self.two_hop_delivery = 0
		self.failures = 0
		self.infinites = 0
		
		self.neighbors = 0