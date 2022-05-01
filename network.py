import math

class Network:

	def dist(self, src, dst):
		return math.sqrt( ((src.x - dst.x)**2) + ((src.y - dst.y)**2) )
		
	def latency(self, d): # 5G wireless link propagation latency per slot
	
		tslot = 50 # in microseconds
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
		T = tslot / Phop

		return T
		
	def delay(self, task, T):
		# packet size 4096 bytes
		# 1 mbits = 1,000,000 bits = 125,000 bytes = 30.51 packets
		# 50 us delay per slot OR packet
		# delay = 50 us * 30.51 = 1525 us = 1.525 ms
		
		return ((task.mbits/8) / 4096) * T