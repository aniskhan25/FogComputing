from __future__ import print_function
import os
import sys
import time
import datetime
import json
import random
from random import choice, randrange, seed
import math
import queue
from sys import platform
import traci
import traci.constants as tc
import numpy as np

if platform == 'linux' or platform == 'linux2':
    sumoBinary = '/usr/bin/sumo'
elif platform == 'darwin':
    sumoBinary = '/Users/anisrahman/sumo/bin/sumo'
elif platform == 'win32':
    sumoBinary = 'C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo'

SIM_STEP_SIZE = 0.1

SIM_STEPS = 1000.0

# -*- args: start -*-

import argparse
  
# Initialize parser
parser = argparse.ArgumentParser()
 
# Adding optional argument
parser.add_argument("-l", "--Lambda", help = "Task Arrival Rate")
parser.add_argument("-m", "--Method", help = "Offloading Selection Method: 0 for Collective (Proposed), 1 for Random, 2 for Nearest, 3 for MinResidence, 4 Epsilon-greedy k-bandit, and 5 Epsilon-decay k-bandit")
 
# Read arguments from command line
args = parser.parse_args()

METHOD = 0
if args.Method:
    METHOD = int(args.Method)

LAMBDA = 1.0
if args.Lambda:
    LAMBDA = float(args.Lambda)

# 1000 iters * 100 vehs * 4 tasks per iter (LAMBDA) = 400,000 tasks --translates-to--> 100 * 4 = 400 task arrival rate

# -*- args: end -*-
 
VEH_RNG = 100
RSU_RNG = 200

TASK_LOWER = 0.2
TASK_UPPER = 1.0

# ARM Cortex R4 (600 MHz; 2011) and R7 (1.0 GHz; 2011); supports dual-core configuration
# https://www.silabs.com/documents/public/white-papers/Which-ARM-Cortex-Core-Is-Right-for-Your-Application.pdf
FREQ_LOWER = 0.6
FREQ_UPPER = 1.0

sumoCmd = [sumoBinary, '-c', 'manhattan.sumocfg', '--no-internal-links', '--step-length', str(SIM_STEP_SIZE)]

lstVeh = {}
lstRsu = {}  
lstPrk = {}  

class Task:
    src = dst = -1

    mbits = dsize = 0.0

    federated = False

    def __init__(this,_src,_mbits,_dsize):
        this.src = _src
        this.mbits = _mbits
        this.dsize = _dsize

class eps_oiv():
    '''
    epsilon-greedy k-bandit problem
    
    Inputs
    =====================================================
    k: number of arms (int)
    eps: probability of random action 0 < eps < 1 (float)
    mu: set the average rewards for each of the k-arms.
        Set to "random" for the rewards to be selected from
        a normal distribution with mean = 0. 
        Set to "sequence" for the means to be ordered from 
        0 to k-1.
        Pass a list or array of length = k for user-defined
        values.
    '''
    
    def __init__(self, k, eps, mu='random'):
        self.k = k
        self.eps = eps
        self.k_reward = np.zeros(k)
        self.reward = 0.0
        self.selected = 0
        
        if mu == 'random':
            # Increase the mean for each arm by one
              self.mu = np.random.normal(0, 1, k)
        elif mu == 'sequence':
            # Increase the mean for each arm by one
            self.mu = np.linspace(0, k-1, k)
        elif type(mu) == list or type(mu).__module__ == np.__name__:
            # User-defined averages            
            self.mu = np.array(mu)

    def pull(self):
        # Generate random number
        p = np.random.rand()

        if self.eps == 0 or p < self.eps:
            # Randomly select an action
            a = np.random.choice(self.k)
        else:
            # Take greedy action
            a = np.argmin(self.mu)

        self.selected = a

        self.reward = -np.random.normal(self.mu[a], 1)

    def run(self):
        self.pull()
            
    def reset(self):
        # Resets results while keeping settings
        self.k_reward = np.zeros(self.k)
        self.reward = 0.0
        self.selected = 0

class eps_decay_bandit:
    '''
    epsilon-decay k-bandit problem
    
    Inputs
    =====================================================
    k: number of arms (int)
    n: number of steps (int)
    mu: set the average rewards for each of the k-arms.
        Set to "random" for the rewards to be selected from
        a normal distribution with mean = 0. 
        Set to "sequence" for the means to be ordered from 
        0 to k-1.
        Pass a list or array of length = k for user-defined
        values.
    '''
    
    def __init__(self, k, n, mu='random'):
        self.k = k
        self.n = n
        self.reward = 0.0
        self.selected = 0
        
        if mu == 'random':
            # Increase the mean for each arm by one
              self.mu = np.random.normal(0, 1, k)
        elif mu == 'sequence':
            # Increase the mean for each arm by one
            self.mu = np.linspace(0, k-1, k)
        elif type(mu) == list or type(mu).__module__ == np.__name__:
            # User-defined averages            
            self.mu = np.array(mu)

    def pull(self):
        # Generate random number
        p = np.random.rand()
        if p < 1 / (1 + self.n / self.k):
            # Randomly select an action
            a = np.random.choice(self.k)
        else:
            # Take greedy action
            a = np.argmin(self.mu)

        self.selected = a

        self.reward = -np.random.normal(self.mu[a], 1)
        
    def run(self):
        self.pull()
            
    def reset(self):
        # Resets results while keeping settings
        self.reward = 0.0
        self.selected = 0

class Edge:

    def __init__(this,_x,_y,_id,_cpu):
        this.x   = _x
        this.y   = _y
        this.id  = _id
        this.cpu = _cpu

        this.qLoc = queue.Queue(maxsize=20000)  
        this.qRes = queue.Queue(maxsize=1000)  

        this.et = this.tt = this.wt = 0.0

        this.rt = 0.0

        this.received = 0
        this.allocRsu = this.allocFed = 0
        this.execRsu  = this.execFed  = 0
        this.cmplRsu  = this.cmplFed  = 0

        this.federated = 0
        this.pending = this.failed = 0

        this.oneHop = this.twoHop = this.threeHop = 0

        this.execution = 0.0

        this.curr = Task(-1,0.0,0.0)

        this.reward = 0.0

    def Recv(this, task):
        if task.federated == False:
            rid = this.CandidateFed()
        else:
            rid = this.id

        if rid != this.id:
            task.federated = True
            oRsu = lstRsu[rid]

            d = math.sqrt((this.x - oRsu.x)** 2 + (this.y - oRsu.y) ** 2)

            oRsu.Recv(task)

            this.tt += (task.dsize / 15.0)*1000 # wired 15 Mbps; delay in ms

            this.federated += 1

        else:
            this.qLoc.put(task)

            this.wt += this.rt
            this.rt += (task.mbits / this.cpu)

            this.pending  += 1

        this.received += 1

    def CandidateFed(this):
        selected = -1
        mn = math.inf
        for k,v in lstRsu.items():  
            if mn > v.rt:  
                mn = v.rt
                selected = k
        return selected

    def OnStep(this):
        if this.execution > 0.0:
            this.execution -= SIM_STEP_SIZE
        else:
            this.rt -= (this.curr.mbits / this.cpu)
            this.et += this.curr.mbits

            this.qRes.put(this.curr)

            if this.curr.src == -1:
                pass
            elif this.curr.federated == False:
                this.execRsu += 1
            else:
                this.execFed += 1

            this.Exec()
            
        this.Delivery()

    def Exec(this):
        if this.qLoc.empty() == True: return

        task = this.qLoc.get()
        this.curr = task

        this.execution = task.mbits / this.cpu  
        
        if task.federated == False:
            this.allocRsu += 1
        else:
            this.allocFed += 1

        this.pending -= 1 

    def Delivery(this):  
        if this.qRes.empty() == True: return

        task = this.qRes.get()

        if task.src == -1: return # logical BUG to avoid empty current task delivered
   
        if task.federated == True:
            this.cmplFed += 1
        else:
            this.cmplRsu += 1

    def DataRate(this, d): # unused
        N = pow(10.0, -3.0)
        Pt = 1.0
        B = 2.0 
        f = 5.9 * pow(10.0, 9.0)
        c = 3.0 * pow(10.0, 8.0)
        lamb = c / f
        Gt = 1.0
        Gr = 1.0
        pi = float(3.14)
        r = 2.5
        L = 128.1 + 37.5 * math.log(d, 10.0)
        G = pow(lamb / pow(4.0 * pi * d, 2.0), 2.0)
        d = pow(d, -r)
        return B * math.log(1.0 + Pt * L / N, 2.0)

class Vehicle:

    def __init__(this,_x,_y,_id,_cpu):
        this.x   = _x
        this.y   = _y
        this.id  = _id
        this.cpu = _cpu

        this.qLoc = queue.Queue(maxsize=20000)  
        this.qRes = queue.Queue(maxsize=1000)  

        this.et = this.tt = this.wt = 0.0

        this.rt = 0.0

        this.generated = this.received = this.pending = 0
        this.allocLoc = this.allocVeh = 0 
        this.execLoc  = this.execVeh  = 0
        this.cmplLoc  = this.cmplVeh  = 0

        this.failed = 0
        this.local = this.offloaded = 0

        this.oneHop = this.twoHop = this.threeHop = 0

        this.localCompleted = 0

        this.execution = 0.0

        this.offmode = False
        this.curr = Task(-1,0.0,0.0)

        this.reward = 0.0

        n = random.random()
#Generate the inter-event time from the exponential distribution's CDF using the Inverse-CDF technique
        this.inter_event_time = -math.log(1.0 - n) / LAMBDA

    def Recv(this, task):
        this.qLoc.put(task)

        this.wt += this.rt
        this.rt += (task.mbits / this.cpu)

        this.pending  += 1
        this.received += 1

    def StrategyCollective(this):
        vid = this.CandidateVeh()
        rid = this.CandidateRsu()
        pid = this.CandidatePrk()

        oLoc = lstVeh[this.id]

        lt = oLoc.rt

        vt = rt = pt = math.inf

        if vid != -1: 
            oVeh = lstVeh[vid]
            vt   = oVeh.rt

        if rid != -1: 
            oRsu = lstRsu[rid]
            rt   = oRsu.rt

        if pid != -1: 
            oPrk = lstPrk[pid]
            pt   = oPrk.rt

        type = 0 # local
        id = this.id
        if vt < lt and vt < rt and vt < pt: 
            type = 1
            id = vid
        if rt < lt and rt < vt and rt < pt:
            type = 2
            id = rid
        if pt < lt and pt < rt and pt < vt:
            type = 3
            id = pid

#        print(type,':',lt,',',vt,',',rt,',',pt)
        return type,id

    def StrategyRandom(this):
        vid = this.RandomVeh()
        rid = this.RandomRsu()
        pid = this.RandomPrk()

        oLoc = lstVeh[this.id]

        lt = oLoc.rt

        vt = rt = pt = math.inf

        options = []
        options.append(0) # local added

        if vid != -1: 
            oVeh = lstVeh[vid]
            vt   = oVeh.rt
            options.append(1)

        if rid != -1: 
            oRsu = lstRsu[rid]
            rt   = oRsu.rt
            options.append(2)

        if pid != -1: 
            oPrk = lstPrk[pid]
            pt   = oPrk.rt
            options.append(3)

        type = 0 # local
        if lt < vt and lt < rt and lt < pt: type = 0
        else: 
           type = random.choice(options)

        id = this.id
        if type == 1: id = vid
        if type == 2: id = rid
        if type == 3: id = pid
 
        return type,id

    def StrategyNearest(this):
        vid = this.NearestVeh()
        rid = this.NearestRsu()
        pid = this.NearestPrk()

        oLoc = lstVeh[this.id]

        lt = oLoc.rt

        vt = rt = pt = math.inf

        options = []
        options.append(0) # local

        mn = math.inf
        nearest = 0

        if vid != -1: 
            oVeh = lstVeh[vid]
            vt   = oVeh.rt
            options.append(1)

            d = math.sqrt((this.x - oVeh.x) ** 2 + (this.y - oVeh.y) ** 2)
            if d < mn:
                nearest = 1
                id = vid
                mn = d

        if rid != -1: 
            oRsu = lstRsu[rid]
            rt   = oRsu.rt
            options.append(2)

            d = math.sqrt((this.x - oRsu.x) ** 2 + (this.y - oRsu.y) ** 2)
            if d < mn: 
                nearest = 2
                id = rid
                mn = d

        if pid != -1: 
            oPrk = lstPrk[pid]
            pt   = oPrk.rt
            options.append(3)

            d = math.sqrt((this.x - oPrk.x) ** 2 + (this.y - oPrk.y) ** 2)
            if d < mn: 
                nearest = 3
                id = pid
                mn = d

        type = 0 # local
        if lt < vt and lt < rt and lt < pt: return type, this.id
        else: return nearest,id

    def StrategyResidence(this):
        vid = this.CandidateVeh()
        rid = this.CandidateRsu()
        pid = this.CandidatePrk()

        oLoc = lstVeh[this.id]

        lt = oLoc.rt

        vt = rt = pt = math.inf

        mn = lt

        id = this.id
        selected = 0

        if vid != -1: 
            oVeh = lstVeh[vid]
            vt   = oVeh.rt

            if vt < mn: 
                selected = 1
                id = vid

        if rid != -1: 
            oRsu = lstRsu[rid]
            rt   = oRsu.rt
    
            if rt < mn: 
                selected = 2
                id = rid

        if pid != -1: 
            oPrk = lstPrk[pid]
            pt   = oPrk.rt

            if pt < mn: 
                selected = 3
                id = pid

        return selected,id

    def StrategyEpsilonGreedy(this):
        vid = this.CandidateVeh()
        rid = this.CandidateRsu()
        pid = this.CandidatePrk()

        oLoc = lstVeh[this.id]

        lt = oLoc.rt
        lr = oLoc.reward

        vt = rt = pt =  math.inf
        vr = rr = pr =  math.inf

        id = this.id
        selected = 0

        choices  = [0]
        waitlist = [lt]
        rewardlist = [lr]

        if vid != -1: 
            oVeh = lstVeh[vid]
            vt   = oVeh.rt
            vr   = oVeh.reward
            choices.append(1)
            waitlist.append(vt)
            rewardlist.append(vr)

        if rid != -1: 
            oRsu = lstRsu[rid]
            rt   = oRsu.rt
            rr   = oRsu.reward
            choices.append(2)
            waitlist.append(rt)
            rewardlist.append(rr)
    
        if pid != -1: 
            oPrk = lstPrk[pid]
            pt   = oPrk.rt
            pr   = oPrk.reward
            choices.append(3)
            waitlist.append(pt)
            rewardlist.append(pr)

        k = len(choices)

        eps = 0.1

        oiv_bandit = eps_oiv(k, eps, mu=waitlist)
        oiv_bandit.k_reward = rewardlist
        oiv_bandit.run()

        type = choices[oiv_bandit.selected]

        if type == 1:
            id = vid
        elif type == 2:
            id = rid
        elif type == 3:
            id = pid
        else:
            id = this.id

        reward = oiv_bandit.reward

        return type,id,reward

    def StrategyEpsilonDecay(this):
        vid = this.CandidateVeh()
        rid = this.CandidateRsu()
        pid = this.CandidatePrk()

        oLoc = lstVeh[this.id]

        lt = oLoc.rt

        vt = rt = pt =  math.inf

        id = this.id
        selected = 0

        choices  = [0]
        waitlist = [lt]
        steps = [this.received]

        if vid != -1: 
            oVeh = lstVeh[vid]
            vt   = oVeh.rt
            choices.append(1)
            waitlist.append(vt)
            steps.append(oVeh.received)

        if rid != -1: 
            oRsu = lstRsu[rid]
            rt   = oRsu.rt
            choices.append(2)
            waitlist.append(rt)
            steps.append(oRsu.received)
    
        if pid != -1: 
            oPrk = lstPrk[pid]
            pt   = oPrk.rt
            choices.append(3)
            waitlist.append(pt)
            steps.append(oPrk.received)

        k = len(choices)

        n = sum(steps)/len(steps)

        oiv_bandit = eps_oiv(k, n, mu=waitlist)
        oiv_bandit.run()

        type = choices[oiv_bandit.selected]

        if type == 1:
            id = vid
        elif type == 2:
            id = rid
        elif type == 3:
            id = pid
        else:
            id = this.id

        reward = oiv_bandit.reward

        return type,id,reward

    def StrategyOptimistic(this):
        vid = this.CandidateVeh()
        rid = this.CandidateRsu()
        pid = this.CandidatePrk()

        oLoc = lstVeh[this.id]

        lt = oLoc.rt

        vt = rt = pt =  math.inf

        id = this.id
        selected = 0

        choices  = [0]
        waitlist = [lt]

        if vid != -1: 
            oVeh = lstVeh[vid]
            vt   = oVeh.rt
            choices.append(1)
            waitlist.append(vt)

        if rid != -1: 
            oRsu = lstRsu[rid]
            rt   = oRsu.rt
            choices.append(2)
            waitlist.append(rt)
    
        if pid != -1: 
            oPrk = lstPrk[pid]
            pt   = oPrk.rt
            choices.append(3)
            waitlist.append(pt)

        k = len(choices)

        eps = 0.0

        oiv_bandit = eps_oiv(k, eps)
        oiv_bandit.run()

        type = choices[oiv_bandit.selected]

        if type == 1:
            id = vid
        elif type == 2:
            id = rid
        elif type == 3:
            id = pid
        else:
            id = this.id

        reward = oiv_bandit.reward

        return type,id,reward

    def Move(this, task):

        reward = 0.0

        if METHOD == 0:
            type,id = this.StrategyCollective()
        elif METHOD == 1:
            type,id = this.StrategyRandom()
        elif METHOD == 2:
            type,id = this.StrategyNearest()
        elif METHOD == 3:
            type,id = this.StrategyResidence()
        elif METHOD == 4:
            type,id,reward = this.StrategyEpsilonGreedy()
        elif METHOD == 5:
            type,id,reward = this.StrategyEpsilonDecay()
        else:
            type,id,reward = this.StrategyOptimistic()

        if type == 0: # local
            this.offmode = False

            task.dst = this.id

            this.qLoc.put(task)

            this.wt += this.rt
            this.rt += (task.mbits / this.cpu)

            this.pending += 1
            this.local   += 1

            this.reward = this.reward + (reward - this.reward) / (this.received + 1)

        elif type == 1: # vehicle
            oVeh = lstVeh[id]
            if oVeh.offmode == True:
                this.offmode = False

                task.dst = this.id

                this.qLoc.put(task)

                this.wt += this.rt
                this.rt += (task.mbits / this.cpu)

                this.pending += 1
                this.local   += 1

                this.reward = this.reward + (reward - this.reward) / (this.received + 1)

            else:
                this.offmode = True

                task.dst = id

                oVeh.Recv(task)

                d = math.sqrt((this.x - oVeh.x)** 2 + (this.y - oVeh.y) ** 2)
                this.tt += this.Delay(task.dsize, this.Latency(d))

                this.offloaded += 1

#                print(oVeh.reward,":",reward)

                oVeh.reward = oVeh.reward + (reward - oVeh.reward) / (oVeh.received + 1)

#                print(oVeh.reward,":",reward)

        elif type == 2: # rsu
            this.offmode = True

            task.dst = id

            oRsu = lstRsu[id]
            oRsu.Recv(task)

            d = math.sqrt((this.x - oRsu.x)** 2 + (this.y - oRsu.y) ** 2)
            this.tt += this.Delay(task.dsize, this.Latency(d))

            this.offloaded += 1

            oRsu.reward = oRsu.reward + (reward - oRsu.reward) / (oRsu.received - oRsu.federated + 1)

        else: # parked
            this.offmode = True

            task.dst = id

            oPrk = lstPrk[id]
            oPrk.Recv(task)

            d = math.sqrt((this.x - oPrk.x)** 2 + (this.y - oPrk.y) ** 2)
            this.tt += this.Delay(task.dsize, this.Latency(d))

            this.offloaded += 1

            oPrk.reward = oPrk.reward + (reward - oPrk.reward) / (oPrk.received + 1)

    def CandidateVeh(this):
        mn = math.inf
        selected = -1

        for k,v in lstVeh.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and this.id != v.id and mn > v.rt:
                mn = v.rt
                selected = k

        return selected

    def CandidateRsu(this):  
        mn = math.inf
        selected = -1

        for k,v in lstRsu.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and mn > v.rt:
                mn = v.rt
                selected = k

        return selected

    def CandidatePrk(this):
        mn = math.inf
        selected = -1

        for k,v in lstPrk.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and mn > v.rt:
                mn = v.rt
                selected = k 

        return selected

    def RandomVeh(this):
        neighbors = [-1]
        for k,v in lstVeh.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and this.id != v.id:
                neighbors.append(k)

        return random.choice(neighbors)

    def RandomRsu(this):
        neighbors = [-1]
        for k,v in lstRsu.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and this.id != v.id:
                neighbors.append(k)

        return random.choice(neighbors)

    def RandomPrk(this):
        neighbors = [-1]
        for k,v in lstPrk.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and this.id != v.id:
                neighbors.append(k)

        return random.choice(neighbors)

    def NearestVeh(this):
        mn = math.inf
        selected = -1

        for k,v in lstVeh.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and this.id != v.id and d < mn:
                mn = d
                selected = k

        return selected

    def NearestRsu(this):  
        mn = math.inf
        selected = -1

        for k,v in lstRsu.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and d < mn:
                mn = d
                selected = k

        return selected

    def NearestPrk(this):
        mn = math.inf
        selected = -1

        for k,v in lstPrk.items():
            d = math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2)
            if d < VEH_RNG and d < mn:
                mn = d
                selected = k

        return selected

    def DataRate(this, d):
        N = pow(10.0, -3.0)  
        Pt = 1.0
        B = 1.0
        f = 5.9 * pow(10.0, 9.0)
        c = 3 * pow(10.0, 8.0)
        lamb = c / f
        Gt = 1.0
        Gr = 1.0
        pi = float(3.14)
        G = pow(lamb / pow(4.0 * pi * d, 2.0), 2.0)
        L = 63.3 + 17.7 * math.log(d, 10.0)
        return B * math.log(1.0 + Pt * L / N, 2.0)

    def OnStep(this):
        if this.execution > 0.0:
            this.execution -= SIM_STEP_SIZE
        else:
            this.rt -= (this.curr.mbits / this.cpu)
            this.et += this.curr.mbits

            this.qRes.put(this.curr)

            if this.curr.src == -1:
                pass
            elif this.curr.src == this.id:
                this.execLoc += 1
            else:
                this.execVeh += 1

            this.Exec()
            
        this.Delivery()

    def Exec(this):
        task = this.qLoc.get()
        this.curr = task

        this.execution = task.mbits / this.cpu  
        
        if task.src == this.id:
            this.allocLoc += 1
        else:
            this.allocVeh += 1

        this.pending -= 1 

    def Delivery(this):  
        if this.qRes.empty() == True: return

        task = this.qRes.get()

        if task.src == -1: return

        if this.id == task.src:
            this.cmplLoc += 1
            return
   
        oVeh = lstVeh[task.src]
        d = math.sqrt((this.x - oVeh.x)**2 + (this.y - oVeh.y)**2)
        if d < VEH_RNG:
            this.oneHop += 1
        elif d >= VEH_RNG and d < 2* VEH_RNG:
            this.twoHop += 1
        else:
            this.failed += 1

        this.cmplVeh += 1

    def Generate(this):
        if this.inter_event_time < 0:
            n = random.random()
            #Generate the inter-event time from the exponential distribution's CDF using the Inverse-CDF technique
            this.inter_event_time = -math.log(1.0 - n) / LAMBDA
			
            task = Task(this.id,random.uniform(TASK_LOWER,TASK_UPPER),random.randint(1, 6))

            this.generated += 1
            this.Move(task)
        this.inter_event_time -= SIM_STEP_SIZE

    def Latency(this,d): # 5G wireless link propagation latency per slot
	
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
		
    def Delay(this,dsize, T):
        # packet size 4096 bytes
        # 1 mbits = 1,000,000 bits = 125,000 bytes = 30.51 packets
        # 50 us delay per slot OR packet
        # delay = 50 us * 30.51 = 1525 us = 1.525 ms
		
        return ((dsize*1000000/8) / 4096) * (T/1000)

# -*- sim: start -*-

id = 0
for x in range(100,901,400):
    for y in range(100,901,400):
        lstRsu[id] = Edge(x, y, id, FREQ_UPPER*2)
        id += 1

for id in range(20):
    lstPrk[id] = Vehicle(randrange(900),randrange(900),id,random.uniform(FREQ_LOWER,FREQ_UPPER))

traci.start(sumoCmd)

step = 0.0
seed(1)  
while step < SIM_STEPS:
    print("Step : ", '{:.0f}'.format(step), end='\r')

    traci.simulationStep()

    for k,v in lstRsu.items():
        if v.qLoc.empty() != True:
            lstRsu[k].OnStep()

    for k,v in lstPrk.items():
        if v.qLoc.empty() != True:
            lstPrk[k].OnStep()

    ids = traci.vehicle.getIDList()

    for id in traci.vehicle.getIDList():
        position = traci.vehicle.getPosition(id)
        if id in lstVeh:
            oVeh = lstVeh[id]
            oVeh.x = position[0]
            oVeh.y = position[1]

        else:
            oVeh = Vehicle(position[0],position[1],id,random.uniform(FREQ_LOWER,FREQ_UPPER))
            lstVeh[id] = oVeh

        oVeh.steps = step

        oVeh.Generate()
        if oVeh.qLoc.empty() != True:
            oVeh.OnStep()

    step += SIM_STEP_SIZE

traci.close()

# -*- sim: end -*-

# -*- stats: start -*-

et = wt = tt = 0.0

etVeh = ttVeh = wtVeh = 0.0
etRsu = ttRsu = wtRsu = 0.0
etPrk = ttPrk = wtPrk = 0.0

generated = 0

recvLoc  =  recvVeh =  recvRsu =  recvPrk =  recvFed = 0
allocLoc = allocVeh = allocRsu = allocPrk = allocFed = 0
execLoc  =  execVeh =  execRsu =  execPrk =  execFed = 0
cmplLoc  =  cmplVeh =  cmplRsu =  cmplPrk =  cmplFed = 0

oneHopVeh = twoHopVeh = failedVeh = 0
oneHopPrk = twoHopPrk = failedPrk = 0

temp = [0]*len(lstVeh)

i = 0
for k,v in lstVeh.items():
    recvLoc += v.local
    recvVeh += v.received

    allocLoc += v.allocLoc
    allocVeh += v.allocVeh

    execLoc += v.execLoc
    execVeh += v.execVeh

    cmplLoc += v.cmplLoc
    cmplVeh += v.cmplVeh

    generated += v.generated

    oneHopVeh   += v.oneHop
    twoHopVeh   += v.twoHop
    failedVeh   += v.failed

    etVeh += v.et
    ttVeh += v.tt
    wtVeh += v.wt

    et += v.et
    wt += v.wt
    tt += v.tt

    temp[i] += (v.local + v.received)
    i +=1

#print(temp)

etVeh = etVeh/(execLoc + execVeh)
ttVeh = ttVeh/(generated - recvLoc)
wtVeh = wtVeh/(recvLoc + recvVeh)

temp = [0]*len(lstRsu)

i = 0
for k,v in lstRsu.items():
    recvRsu += v.received
    recvFed += v.federated

    allocRsu += v.allocRsu
    allocFed += v.allocFed

    execRsu += v.execRsu
    execFed += v.execFed

    cmplRsu += v.cmplRsu
    cmplFed += v.cmplFed

    etRsu += v.et
    ttRsu += v.tt
    wtRsu += v.wt

    et += v.et
    wt += v.wt
    tt += v.tt

    temp[i] += (v.received - v.federated)
    i +=1

#print(temp)

etRsu = etRsu/(execRsu + execFed)
ttRsu = ttRsu/recvFed
wtRsu = wtRsu/recvRsu # recvRsu accounts for both Rsu and Fed

temp = [0]*len(lstPrk)

i = 0
for k,v in lstPrk.items():
    recvPrk += v.received

    allocPrk += v.allocVeh

    execPrk += v.execVeh

    cmplPrk += v.cmplVeh

    oneHopPrk   += v.oneHop
    twoHopPrk   += v.twoHop
    failedPrk   += v.failed

    etPrk += v.et
    ttPrk += v.tt # should be zero as Prk is not offloading
    wtPrk += v.wt

    et += v.et
    wt += v.wt
    tt += v.tt

    temp[i] += v.received
    i +=1

#print(temp)

etPrk = etPrk/execPrk
wtPrk = wtPrk/recvPrk

#executed = execLoc + execVeh + execRsu + execFed + execPrk
#received = recvVeh + recvRsu + recvFed + recvPrk

totalNodes = len(lstVeh) + len(lstRsu) +len(lstPrk)

print('Rate (tasks per sec): ', '{:.1f}'.format(LAMBDA*len(lstVeh)))

#ttRsu = ttRsu/recvFed
print('Avg     exec time (ms): ', '{:.3f}'.format((etVeh/len(lstVeh) + etRsu/len(lstRsu) + etPrk/len(lstPrk))/3*1000))
print('Avg transmit time (ms): ', '{:.3f}'.format((ttVeh/len(lstVeh) + ttRsu/len(lstRsu) + ttPrk/len(lstPrk))/3))
print('Avg     wait time (ms): ', '{:.3f}'.format((wtVeh/len(lstVeh) + wtRsu/len(lstRsu) + wtPrk/len(lstPrk))/3*1000))

print('    Exec time (s): ', '{:.3f}'.format(et))
print('Transmit time (s): ', '{:.3f}'.format(tt/1000))
print('    Wait time (s): ', '{:.3f}'.format(wt))

print('Tasks generated: ', generated)

print(' Tasks received (loc): ', recvLoc)
print(' Tasks received (veh): ', recvVeh)
print(' Tasks received (rsu): ', recvRsu - recvFed)
print(' Tasks received (fed): ', recvFed)
print(' Tasks received (prk): ', recvPrk)

print(' Tasks allocated (loc): ', allocLoc)
print(' Tasks allocated (veh): ', allocVeh)
print(' Tasks allocated (rsu): ', allocRsu)
print(' Tasks allocated (fed): ', allocFed)
print(' Tasks allocated (prk): ', allocPrk)

print(' Tasks executed (loc): ', execLoc)
print(' Tasks executed (veh): ', execVeh)
print(' Tasks executed (rsu): ', execRsu)
print(' Tasks executed (fed): ', execFed)
print(' Tasks executed (prk): ', execPrk)

print(' Tasks completed (loc): ', cmplLoc)
print(' Tasks completed (veh): ', cmplVeh)
print(' Tasks completed (rsu): ', cmplRsu)
print(' Tasks completed (fed): ', cmplFed)
print(' Tasks completed (prk): ', cmplPrk)

print('  One-hop (veh): ',   oneHopVeh)
print('  Two-hop (veh): ',   twoHopVeh)

print('  One-hop (prk): ',   oneHopPrk)
print('  Two-hop (prk): ',   twoHopPrk)

print('Failed (veh): ', failedVeh)
print('Failed (prk): ', failedPrk)

print('Throughput (Mbits/s): ', '{:.3f}'.format(et/SIM_STEPS))

# -*- stats: end -*-

# -*- utilization stats: start -*-

temp = [0]*len(lstVeh)

i = 0
for k,v in lstVeh.items():
    temp[i] += (v.local + v.received)
    i +=1

#print(temp)

temp = [0]*len(lstRsu)

i = 0
for k,v in lstRsu.items():
    temp[i] += (v.received - v.federated)
    i +=1

#print(temp)

temp = [0]*len(lstPrk)

i = 0
for k,v in lstPrk.items():
    temp[i] += v.received
    i +=1

#print(temp)

# -*- utilization stats: end -*-


