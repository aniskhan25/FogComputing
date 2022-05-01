from typing import Any

import os
import sys
import optparse

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
from sumolib import checkBinary  # Checks for the binary in environ vars
import traci
import traci.constants as tc
import math
import queue 
import threading
from random import seed

#import random

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

#The task is to reduce the execution time(this.execution) check UpdateExecutionTimer() method

#sumoBinary = "C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo-gui"
if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
        sumoBinary = "C:\\sumo-1.6.0\\bin\\sumo-gui"
        sumoCmd = [sumoBinary, "-c", "manhattan.sumocfg"]
        traci.start(sumoCmd)



Manager = {}  # create empty dictionary to store the key value of every vehicle
DataHolder = [] # create empty list to hold the 
RSUManager = {}  # manage RSU
ParkManager = {}  # manage Parked Vehicles


class EdgeCompute:  # this class is under construction, purpose is to use the federation approach here
    def __init__(this, _xloc, _yloc,_id, _range, _cpu):
        this.x_axis = _xloc
        this.y_axis = _yloc
        this.id = _id
        this.range = _range
        this.cpu = _cpu
        
        this.Collection= {}
        this.nearbyRSUtoRSU = {}
        this.nearbyPark = {}
        this.rsuList = {}
        
        
        this.que = queue.Queue(maxsize=9000) # This is to hold the tasks for execution
        this.queRSUtoRSU = queue.Queue(maxsize=9000) # This is to hold the tasks for execution
        this.resultDel = queue.Queue(maxsize=500) # This is to hold result for delivery
        this.resultDelRSUtoRSU = queue.Queue(maxsize=4000) # This is to hold result for delivery
        this.waitingTime = 0
        
        this.executionTime = 0
        this.LocalAvgWT =0 # local node average waiting time
        this.direct_delivery = 0
        this.oneHopDlv =0
        this.twoHopDlv =0
        this.threeHopDlv = 0
        this.fourHopDlv = 0
        this.fiveHopDlv = 0
        this.failure = 0
        this.steps = 0.0 # time simulation
        this.countLocalExec = 0
        this.execution = 0
        this.totalTskRecv = 0
        
        this.RSUHandoff =0
        this.taskFederated = 0
        this.FederatedTskReceived = 0
        this.task_returnedRSU = 0
        this.AvgWaitintTimeRSU = 0
        this.Total_recvRsuToRsu =0
        this.Total_recv = 0
        this.AvgWaitTimeRSU = 0
        this.offloadTskRSU = 0
        this.totalTskRecvRSUtoRSU =0
        this.countTotalExec = 0                
        this.direct_deliveryRSUtoRSU = 0
        this.oneHopDlvRSUtoRSU = 0
        this.twoHopDlvRSUtoRSU = 0
        this.threeHopDlvRSUtoRSU = 0   
        this.fourHopDlvRSUtoRSU =0
        this.fiveHopDlvRSUtoRSU = 0       
        this.failureRSUtoRSU = 0      
        this.LocalAvgWTRsuToRsu = 0
        this.AvgWaitintTimeRsuToRsu = 0
        
        this.waitingTimeRSUtoRSU = 0
        this.executionTimeRSUtoRSU = 0
        this.countTotalExecRSUtoRSU = 0
        this.TransmissionTime = 0
        this.LocalAvgTT = 0
        this.TransmissionTimeRSUtoRSU = 0
        this.LocalAvgTTRsutoRsu = 0
        this.rateV2RSU = 0
        this.rateV2RSUtoRSU = 100
        #this.handoff =0
        
        

    def OffloadToRSU(this, _task):
     size = len(this.nearbyRSUtoRSU)
     #print ("in EdgeCC is outside if ", size)
     if size > 0:   # if rsu within range
        #print ("in EdgeCC is inside if ", size)
        dest = list(this.nearbyRSUtoRSU)[0]   # or first RSU selected 
        #print ("Dest RSU is", dest)
        _task.DID = dest
        
        DestRSU_object = RSUManager[dest]
        DestRSU_object.RecvTask(_task)
        #this.offloadTskRSU = this.offloadTskRSU + 1
        this.RSUHandoff = this.RSUHandoff + 1
        RSUManager[dest] = DestRSU_object  
        obj = this.nearbyRSUtoRSU[dest]
        this.BuildDataset(_task, obj.distance, obj.wtme ,"V2RSU") # 2
        #print ("Task send from ", this.id, " to RSU ", dest)
     else:
        #print ("in EdgeCC local queue", size)
        this.AddLocalQueue(_task)	 
         
        
    def OffloadInRSUFederation(this, _tsk, distance_val):
       selected_rsu = this.id
       #handoff = 0
       for Kk, Vv in RSUManager.items(): # runs 6 time #Kk is the RSU id and Vv object of edge compute...
           if this.waitingTime > Vv.waitingTimeRSUtoRSU: # and Vv.waitingTime > Vv.waitingTimeRSUtoRSU:   
             #if this.waitingTime > Vv.waitingTimeRSUtoRSU:  
             selected_rsu = Vv.id  
             #RSU_obj = Manager[_tsk.SID]
             #selected_rsu = RSU_obj.RSUSelectionDistanceBased(selected_rsu)  
            
            
       if selected_rsu != this.id: # it means it is equal to Vv.id not this.id, vV.id is any RSU selected from the list of RSU with the less waiting time
           _tsk.RSUFed = True   # only one time sharing (Add to lcal queue)
           this.handoff= 1
           obj = RSUManager[selected_rsu]     
           obj.RecvTask(_tsk, distance_val)
           obj.FederatedTskReceived = obj.FederatedTskReceived + 1
           RSUManager[selected_rsu] = obj
           this.taskFederated = this.taskFederated + 1
           return True
       else: 
           this.handoff = 0
           return False
               

    def AddLocalQueueRsuToRsu(this, _tsk):
      this.LocalAvgWTRsuToRsu = this.LocalAvgWTRsuToRsu + this.waitingTimeRSUtoRSU
      this.queRSUtoRSU.put(_tsk)
      this.ComputeWaitingTimeRsuToRsu(_tsk)
      #this.ComputeWaitingTime(_tsk) 
      #this.AverageWaitingTime_RsuToRsu(_tsk)
      #this.BuildDataset(_tsk,0.0,0.0, 0)
 

 
      
    def AddLocalQueue(this, _tsk):
      this.LocalAvgWT = this.LocalAvgWT + this.waitingTime
      this.que.put(_tsk)
      this.ComputeWaitingTime(_tsk)
      #this.AverageWaitingTime_RSU(_tsk)
      #this.BuildDataset(_tsk,0.0,0.0, 0)     

    def  ComputeTransmissionTime(this, _tsk):
     print("task.data",_tsk.dataSize )
     this.TransmissionTime = this.TransmissionTime + _tsk.dataSize/this.rateV2RSU

         
    def  ComputeWaitingTime(this, _tsk):
     this.waitingTime = this.waitingTime + (_tsk.MIPS/this.cpu)
     
    def  AverageWaitingTime_RSU(this, _tsk): #Average Waiting time for this RSU
     #this.LocalAvgWT = this.LocalAvgWT + this.waitingTime     
     this.AvgWaitintTimeRSU= this.AvgWaitintTimeRSU + this.LocalAvgWT
     this.Total_recv = this.Total_recv + this.totalTskRecv     
     this.AvgWaitTimeRSU =(this.AvgWaitintTimeRSU/(this.Total_recv + 0.1))     
     
    def  ComputeTransmissionTimeRsuToRsu(this, _tsk):
     this.TransmissionTimeRSUtoRSU = this.TransmissionTimeRSUtoRSU + _tsk.dataSize/this.rateV2RSUtoRSU

     
    def  ComputeWaitingTimeRsuToRsu(this, _tsk):
     this.waitingTimeRSUtoRSU = this.waitingTimeRSUtoRSU + (_tsk.MIPS/this.cpu)     
 
    def  AverageWaitingTime_RsuToRsu(this, _tsk): #Average Waiting time for this RSU
     #this.LocalAvgWT = this.LocalAvgWT + this.waitingTime     
     this.AvgWaitintTimeRsuToRsu= this.AvgWaitintTimeRsuToRsu + this.LocalAvgWTRsuToRsu
     this.Total_recvRsuToRsu = this.Total_recvRsuToRsu + this.totalTskRecvRSUtoRSU    
     this.AvgWaitintTimeRsuToRsu =(this.AvgWaitintTimeRsuToRsu/(this.Total_recvRsuToRsu + 0.1))  


    def ExecuteTask(this):
     # print("ExecuteTask( in edge compute")
      if this.que.empty() != True:
        tpk = this.que.get()
        #this.execution = round(tpk.MIPS/this.cpu)      # execution time of current task
        this.execution = tpk.MIPS/this.cpu      # execution time of current task
        this.executionTime = this.executionTime + this.execution  # measure total execution time at each RSU
        this.waitingTime = this.waitingTime - this.execution  # compute waiting time
        this.countLocalExec =  this.countLocalExec + 1
        this.resultDel.put(tpk)
        this.ResultDelivery(tpk)

        
    def ExecuteTaskRSUtoRSU(this):
     # print("ExecuteTask( in edge compute")
      if this.queRSUtoRSU.empty() != True:
        tpk = this.queRSUtoRSU.get()
        #this.execution = round(tpk.MIPS/this.cpu)      # execution time of current task
        this.execution = tpk.MIPS/this.cpu      # execution time of current task
        this.executionTimeRSUtoRSU = this.executionTimeRSUtoRSU  + this.execution  # measure total execution time at each RSU
        this.waitingTimeRSUtoRSU = this.waitingTimeRSUtoRSU - this.execution  # compute waiting time
        this.countTotalExecRSUtoRSU =  this.countTotalExecRSUtoRSU + 1   
        this.resultDelRSUtoRSU.put(tpk)       
        this.ResultDeliveryRSUtoRSU(tpk)        
        

    def UpdateExecutionTimer(this):
     #if this.execution !=0 and this.execution > 0.3:  #task still executing, see ExecuteTask function
     if this.execution >  0:
       this.execution = this.execution - 1           
     else: 
      this.ExecuteTaskRSUtoRSU()
      this.ExecuteTask()
      
      



    def V2RSUDatarate(this,distance_val):
   
         #p = pow(10, -9)#( in watt
         N =  pow(10, -3)
         Pt = 1 # W(in watt) # pt is the transmission power of the vehicles.
         #B= 20* pow(10, 6)
         B= 2 # result in MHZ
         f= 5.9 * pow(10, 9)
         c = 3* pow(10,8)
         lamb = c/f
         Gt = 1
         Gr = 1
         pi = float(3.14)
         r=2.5
         #d= 20
         #Pr = Pt*Gt*Gr*(lamb/(4*pi*d)^2
         L = 128.1 + 37.5*math.log(distance_val, 10) 
         G = pow((lamb/(pow(4*pi*distance_val, 2))),2)
         
         #G = lamb/pow(4*pi*distance_val, 2)
         print("G", G)
         print("distance", distance_val)
         
         d = pow(distance_val,-r)     
         #this.rateV2RSU = B * math.log((1+(Pt*d*G)/(p)),2)
         this.rateV2RSU = B * math.log((1+(Pt*L)/(N)),2)
         print("rateV2RSU", this.rateV2RSU)    
   
   
   
   
    def Extra(this,distance_val): 
   
      N =  pow(10, -9)#( in watt
      Pt = 1 # W(in watt) # pt is the transmission power of the vehicles.
      B= 1
    #  B= 20 # result will be in MHZ
      f= 5.9 * pow(10, 9)
      c = 3* pow(10,8)
      lamb = c/f
      Gt = 1
      Gr = 1
      pi = float(3.14)
      #d= 20
      #Pr = Pt*Gt*Gr*(lamb/(4*pi*d)^2
      G = pow((lamb/(pow(4*pi*distance_val, 2))),2)
      #G = lamb/pow(4*pi*distance_val, 2)
      print("distance", distance_val)
      print("G", G)   
     # L= pow(10, -(63.3 + 17.7*math.log(distance_val,10))/10)
      L = 63.3 + 17.7*math.log(distance_val,10)
      print("L", L)
      #this.rateV2V = B * math.log((1+(Pt*L*G)/(N)),2)
      this.rateV2V = B * math.log((1+(Pt*L)/(N)),2)
      print("rateV2Vcomm", this.rateV2V) 
   

    def RecvTask(this,_task, distance_val):
        
     if _task.RSUFed == False: 
       if this.OffloadInRSUFederation (_task, distance_val) != True:
         this.LocalAvgTT = this.LocalAvgTT + this.TransmissionTime 
         this.V2RSUDatarate(distance_val)      
         this.ComputeTransmissionTime(_task)
         this.AddLocalQueue(_task)
         this.totalTskRecv = this.totalTskRecv + 1   
     elif _task.RSUFed == True : # For RSU to RSU offload         
         #this.AddLocalQueue(_task)        
         this.LocalAvgTTRsutoRsu = this.LocalAvgTTRsutoRsu + this.TransmissionTimeRSUtoRSU                  
         this.ComputeTransmissionTimeRsuToRsu(_task)
         this.AddLocalQueueRsuToRsu(_task)
         this.totalTskRecvRSUtoRSU = this.totalTskRecvRSUtoRSU + 1   
                 
         

    def ResultDelivery(this, _inExecTsk): # EdgeCompute class /
     if this.resultDel.qsize() > 0: # Maximum size of this.resultDel is 100 as defined, hold the results
       #print ("this.resultDel.qsize() Result deliver edge compute is:=",this.resultDel.qsize())
       _tt = this.resultDel.get()  # gets and holds the result for this task (Task in the execution delivery class)
       #print (" this.x_axis", this.x_axis)
       #print (" this.y_axis", this.y_axis)
       #print ("Id of RSU\n\n\n\n\n\n\n\n\n\\n\n\n\n\n\n",this.id)
       SrcVeh_object = Manager[_tt.SID] # Get source vehicle ID (Perhaps Object)from the Manger Class, from the complete list that describes 100 vehicles.
       dlv_dist = math.sqrt( ((this.x_axis - SrcVeh_object.x_axis)**2)+((this.y_axis - SrcVeh_object.y_axis)**2))
       if dlv_dist > 0 and dlv_dist < 150:
         this.direct_delivery = this.direct_delivery + 1
         #print (" direct delivery called ", this.direct_delivery)
       elif dlv_dist >= 150 and dlv_dist < 300:
         this.oneHopDlv = this.oneHopDlv + 1
         #print (" 1 HP delivery called ", this.oneHopDlv)
       elif dlv_dist >= 300 and dlv_dist <= 450:
         this.twoHopDlv = this.twoHopDlv + 1
       elif dlv_dist >= 450 and dlv_dist < 600:
           this.threeHopDlv = this.threeHopDlv + 1     

       elif dlv_dist >= 600 and dlv_dist <= 750:
           this.fourHopDlv = this.fourHopDlv + 1  
                 
       else:
         this.failure = this.failure + 1
       this.TaskSuccesfullyReturned(_inExecTsk) 
     else:
         #print("result delivery queue empty")
         this.resultDel.put(_inExecTsk) #Putting task(result) in the resultDel queue
         
         
         
         
         
    def ResultDeliveryRSUtoRSU(this, _inExecTsk): # EdgeCompute class /
     if this.resultDelRSUtoRSU.qsize() > 0: # Maximum size of this.resultDel is 100 as defined, hold the results
       #print ("this.resultDel.qsize() Result deliver edge compute is:=",this.resultDelRSUtoRSU.qsize())
       _tt = this.resultDelRSUtoRSU.get()  # gets and holds the result for this task (Task in the execution delivery class)
       #print (" this.x_axis", this.x_axis)
       #print (" this.y_axis", this.y_axis)
       SrcVeh_object = Manager[_tt.SID] # Get source vehicle ID (Perhaps Object)from the Manger Class, from the complete list that describes 100 vehicles.
       dlv_dist = math.sqrt( ((this.x_axis - SrcVeh_object.x_axis)**2)+((this.y_axis - SrcVeh_object.y_axis)**2))
       if dlv_dist > 0 and dlv_dist < 150:
         this.direct_deliveryRSUtoRSU = this.direct_deliveryRSUtoRSU + 1
         #print (" direct delivery called ", this.direct_delivery)
       elif dlv_dist >= 150 and dlv_dist < 300:
         this.oneHopDlvRSUtoRSU = this.oneHopDlvRSUtoRSU + 1
         #print (" 1 HP delivery called ", this.oneHopDlv)
       elif dlv_dist >= 300 and dlv_dist <= 450:
         this.twoHopDlvRSUtoRSU = this.twoHopDlvRSUtoRSU + 1
       elif dlv_dist >= 450 and dlv_dist < 600:
           this.threeHopDlvRSUtoRSU = this.threeHopDlvRSUtoRSU + 1                
       elif dlv_dist >= 600 and dlv_dist <= 750:
           this.fourHopDlvRSUtoRSU = this.fourHopDlvRSUtoRSU + 1                              
       else:
         this.failureRSUtoRSU = this.failureRSUtoRSU + 1
       this.TaskSuccesfullyReturned(_inExecTsk) 
     else:
         #print("result delivery queue empty")
         this.resultDelRSUtoRSU.put(_inExecTsk) #Putting task(result) in the resultDel queue         




         
    
    def TaskSuccesfullyReturned(this, _inExecTsk): 
     #print(this.id, "Executed Event ...!\n") 
      this.task_returnedRSU = this.direct_delivery + this.oneHopDlv + this.twoHopDlv 

    
    def PrintStatRSU(this):
     print(this.id, " local compute ", this.countLocalExec, " Tsk Recv", this.totalTskRecv, " Pending Tsk ", this.que.qsize(), " Waiting Time = ", this.waitingTime, " Total execution time ", this.executionTime, " Task Federated ", this.taskFederated, " Federted task Recv ", this.FederatedTskReceived)
  


class VehInfo: # Class is used during vehicle selection process based on distance or waiting time, add more selection features which will be available in main class automatically
    def __init__(this,distance, wTime):
     this.distance = distance
     this.wtme = wTime
     
class RSUInfo: # Class is used during vehicle selection process based on distance or waiting time, add more selection features which will be available in main class automatically
    def __init__(this,distance, wTime):
     this.distance = distance
     this.wtme = wTime
    
class Dataset:   # class is used to create a dataset to perform DRL
    #def __init__(this, time, mips, sid, did, swtime, dwtime, range, _dist, dec, t_Gen, l_tasks, v2v_task, f_v2v, f_v2RSU, t_returnedRSU,t_returnedpark, avg_wtimeVeh, avg_wtimeRSU):
             
    # this.time = time
     
     ###Task Profile####
    # this.mips = mips
    
    # this.Sid = sid
    # this.did = did
    # this.s_waitTime = swtime
    # this.d_waitTime = dwtime
    # this.dist = _dist
    # this.inRangeVEH = range                  
    # this.decision = dec # 0 means local and 1 means offload to vehicle 2 mean offload to RSU, 3 to parked                                                          
    # this.total_task_Generated = t_Gen
    # this.local_executiontasks = l_tasks
    # this.v2v_offloadingTasks = v2v_task
    # this.v2v_failureDel =f_v2v
    # this.v2RSU_failureDel = f_v2RSU
    # this.task_returned_rsu = t_returnedRSU
    # this.task_returned_park = t_returnedpark
   #  this.Avg_waitTimeVeh = avg_wtimeVeh
    # this.Avg_waitTimeRSU = avg_wtimeRSU
              

    #def __init__(this, time, mips, dataSize, cpuCycle,tRateV2V,tRateV2Fog,tRateV2Park,relayTask, sid, did, swtime, dwtime, range, _dist, avg_wtimeVeh, avg_wtimeRSU,avg_wtimeRSUtoRSU,handoff,dec):
    def __init__(this, time, mips, dataSize, cpuCycle,tRateV2V,tRateV2Fog,tRateV2Park,relayTask, swtime, dwtime, n_waitV2V,n_waitV2R,n_waitV2P, n_transV2V,n_transV2R,n_transV2P,s_speed,d_speed, range, _dist,handoff,dec): 
     
     this.time = time
     
     ###Task Profile####
     this.mips = mips
     this.dataSize = dataSize
     
     ## Current Network State###
     this.cpuCycle = cpuCycle
     this.tRateV2V = tRateV2V
     this.tRateV2Fog = tRateV2Fog
     this.tRateV2Park = tRateV2Park
     this.relayTask  = relayTask     
     #this.Sid = sid
     #this.did = did
     this.s_waitTime = swtime
     this.d_waitTime = dwtime
     
     ## Waiting time of nearest nodes all 
     this.n_waitV2V = n_waitV2V
     this.n_waitV2R = n_waitV2R
     this.n_waitV2P = n_waitV2P
      
     this.n_transV2V = n_transV2V
     this.n_transV2R = n_transV2R
     this.n_transV2P = n_transV2P
     this.s_speed = s_speed
     this.d_speed = d_speed
     this.inRangeVEH = range      
     this.dist = _dist                                                                           
     #this.Avg_waitTimeVeh = avg_wtimeVeh
     #this.Avg_waitTimeRSU = avg_wtimeRSU  
     #this.Avg_waitTimeRSUtoRSU = avg_wtimeRSUtoRSU
     this.handoff = handoff
     this.decision = dec # 0 means local and 1 means offload to vehicle 2 mean offload to RSU, 3 to parked 

######Speed is missed we will introdue it at the ends

       
class Task: 
    MIPS = 0
    SID  = 0
    DID  = 0
    dataSize =0
    
    #rateV2V = 30 
    #rateV2Park = 40 
    #rateV2RSU = 50
    
    #def __init__(this, mips, source_vid, dest_vid, dataSize, rateV2V, rateV2Park, rateV2RSU, deadlineLocal,deadlineV2V, deadlineV2Park, deadlineV2RSU, deadLine,  f1alse, f2alse):
    #def __init__(this, mips, source_vid, dest_vid, deadLine,  f1alse, f2alse, f3alse):
    
    def __init__(this, mips, source_vid, dest_vid, deadLine, datasize, f1alse, f2alse, f3alse):
        
     this.MIPS = mips
     this.SID =  source_vid
     this.DID =  dest_vid
     #this.size = dataSize
     this.dLine = deadLine
     this.dataSize = datasize
     this.RSUFed = f1alse
     this.ParkFed = f2alse   
     this.localComp = f3alse
     #this.dRateV2V = rateV2V
     #this.dRateV2Park = rateV2Park
     #this.dRateV2RSU = rateV2RSU    
     #this.deadLocal = deadlineLocal
     #this.deadV2V = deadlineV2V
     #this.deadV2Park = deadlineV2Park
     #this.deadV2RSU =   deadlineV2RSU
     
   

class P_Vehicle:  # this class is under construction, purpose is to use the federation approach here
    def __init__(this, _xloc, _yloc,_id, _range, _cpu):
        this.x_axis = _xloc
        this.y_axis = _yloc
        this.id = _id
        this.range = _range
        this.cpu = _cpu
        this.que = queue.Queue(maxsize=9000) # This is to hold the tasks for execution
        this.resultDel = queue.Queue(maxsize=500) # This is to hold result for delivery
        this.waitingTime = 0
        this.executionTime = 0
        this.LocalAvgWT =0 # local node average waiting time
        this.direct_delivery = 0
        this.oneHopDlv =0
        this.twoHopDlv =0
        this.threeHopDlv = 0
        this.fourHopDlv = 0 
        this.fiveHopDlv = 0      
        this.failure = 0
        this.steps = 0.0 # time simulation
        this.countLocalExec = 0
        this.execution = 0
        this.totalTskRecv = 0
        this.taskFederated = 0
        this.FederatedTskReceived = 0
        this.AvgWaitintTimePark =0
        this.task_returnedPark = 0
        this.Total_recv = 0
        this.AvgWaitTimeV2V = 0 # this is v2Park
        this.AvgWaitintTimeVehicle = 0 # parked vehicle
        this.TransmissionTime   = 0
        this.LocalAvgTT = 0
        this.rateV2Park = 0
     
        
       

    def WithInRange_Park(this): #Distance of that particular RSU which wants to offload tasks is calculated from all 6 parked vehicles one by one.(val.id = One park vehicle)

     for key, val in ParkManager.items():
       distance = math.sqrt( ((this.x_axis - val.x_axis)**2)+((this.y_axis - val.y_axis)**2) )
       #if distance < 70: Original Sir Asad Code, Check it
       #if distance < 70:   
       if distance != 0 and distance < 70 and this.id != val.id:
         #print ("Distance of vehicle this. id from RSU is", this.id, distance, this.range) # Note: this.id is "vehicle" and val.id is "RSU"
         park = ParkInfo (distance, val.waitingTime)
         this.nearbyPark[val.id] = park
     

        
     # This fucntion only tells wheter this.id(Currennt RSu) is selected or Vv.id(RSU from the RSUManager) list is selected based on waiting time. Distance is previously checked.
     # Remember if distance is less and waiting time is also less then it is good decision but if  distance is less but waiting time is not small then we will consider the waiting time, distance will not be checked.      
    def OffloadInParkFederation(this, _tsk, distance_val ): # Check which RSU needs to be selected Based on waiting time. Distance is already checked then later on waiting time is the true determining factor
       selected_Park = this.id
       #print ("this. id", "vv.id", this.id, Vv.id)
       for Kk, Vv in ParkManager.items(): # runs 6 time 
           #print ("this. id", "vv.id", this.id, Vv.id) #Vv.id is any RSU selected from the list. Can be different waiting times because different tasks could be pending on an RSU affecting the waiting time. 
           if this.waitingTime > Vv.waitingTime: 
               selected_Park = Vv.id
               #Park_obj = Manager[_tsk.SID]
               #selected_park = Park_obj.ParkSelectionDistanceBased(selected_park) 
        
       if selected_Park != this.id: # it means if selected RSU is Vv.id not this.id, vV.id is any RSU selected from the list of RSU with the less waiting time
           _tsk.ParkFed = True   # only one time sharing (Add to lcal queue)
           obj = ParkManager[selected_Park]     
           obj.RecvTask(_tsk, distance_val)
           obj.FederatedTskReceived = obj.FederatedTskReceived + 1
           ParkManager[selected_Park] = obj
           this.taskFederated = this.taskFederated + 1
           return True
       else: return False
               

    def AddLocalQueue(this, _tsk):
      this.LocalAvgWT = this.LocalAvgWT + this.waitingTime
      this.que.put(_tsk)
      this.ComputeWaitingTime(_tsk)
      #this.AverageWaitingTime_PV(_tsk)
      #this.BuildDataset(_tsk,0.0,0.0, 0)
 
    def  ComputeWaitingTime(this, _tsk): # Waiting time of this parked vehicle
     this.waitingTime = this.waitingTime + (_tsk.MIPS/this.cpu)
     
     
    def  ComputeTransmissionTime(this, _tsk):
     this.TransmissionTime = this.TransmissionTime + _tsk.dataSize/this.rateV2Park     
     
     
    def  AverageWaitingTime_PV(this, _tsk):# Average Waiting time of this parked vehicle
     #this.LocalAvgWT = this.LocalAvgWT + this.waitingTime     
     this.AvgWaitintTimeVehicle= this.AvgWaitintTimeVehicle + this.LocalAvgWT
     this.Total_recv = this.Total_recv + this.totalTskRecv     
     this.AvgWaitTimeV2V =(this.AvgWaitintTimeVehicle/(this.Total_recv+1))
      
    def ExecuteTask(this):
      #print("ExecuteTask( in Park Vehicles")
      if this.que.empty() != True:
        tpk = this.que.get()
        #this.execution = round(tpk.MIPS/this.cpu)      # execution time of current task
        this.execution = tpk.MIPS/this.cpu      # execution time of current task
        this.executionTime = this.executionTime + this.execution  # measure total execution time at each veh
        this.waitingTime = this.waitingTime - this.execution  # compute waiting time
        this.countLocalExec =  this.countLocalExec + 1
        this.resultDel.put(tpk)
        this.ResultDelivery(tpk)
        

    def UpdateExecutionTimer(this):
     #if this.execution !=0 and this.execution > 0.3:  #task still executing, see ExecuteTask function
     if this.execution >  0:
       this.execution = this.execution - 1           
     else: 
       this.ExecuteTask()
 
    def V2ParkDatarate(this,distance_val):
    

         #p = pow(10, -9)#( in watt
         N =  pow(10, -3)
         Pt = 1 # W(in watt) # pt is the transmission power of the vehicles.
         #B= 20* pow(10, 6)
         B= 2 # result in MHZ
         f= 5.9 * pow(10, 9)
         c = 3* pow(10,8)
         lamb = c/f
         Gt = 1
         Gr = 1
         pi = float(3.14)
         r=2.5
         #d= 20
         #Pr = Pt*Gt*Gr*(lamb/(4*pi*d)^2
         L = 128.1 + 37.5*math.log(distance_val, 10) 
         G = pow((lamb/(pow(4*pi*distance_val, 2))),2)
         
         #G = lamb/pow(4*pi*distance_val, 2)
         print("G", G)
         print("distance", distance_val)
         
         d = pow(distance_val,-r)     
         #this.rateV2RSU = B * math.log((1+(Pt*d*G)/(p)),2)
         this.rateV2Park = B * math.log((1+(Pt*L)/(N)),2)
         print("v2Park", this.rateV2Park) 
   

 #   def RecvTask(this,_task, distance_val):
 #    if _task.ParkFed == False: 
       #if this.OffloadInParkFederation (_task) == False:
  #     if this.OffloadInParkFederation (_task, distance_val) != True:   # If this. id (Current RSU) in the "OffloadInParkFederation" is selected, Add or receive task task in this.id local queue
     #    this.LocalAvgTT = this.LocalAvgTT + this.TransmissionTime
     #    this.V2ParkDatarate(distance_val)
     #    this.ComputeTransmissionTime(_task)
     #    this.AddLocalQueue(_task) # Task received in this.id local queue
     #    this.totalTskRecv = this.totalTskRecv + 1   
  #   elif _task.ParkFed == True:  # If Vv.id (RSU from the RSUManager list) in the "OffloadInParkFederation" is selected, Add or receive task task in Vv.id local queue
   #      this.ComputeTransmissionTime(_task)
    #     this.AddLocalQueue(_task) # Task received in Vv.id local queue.
    #     this.totalTskRecv = this.totalTskRecv + 1   
         
    def RecvTask(this,_task, distance_val):
     if _task.ParkFed == False: 
       #if this.OffloadInParkFederation (_task) == False:
       if this.OffloadInParkFederation (_task, distance_val) != True:   # If this. id (Current RSU) in the "OffloadInParkFederation" is selected, Add or receive task task in this.id local queue
         this.LocalAvgTT = this.LocalAvgTT + this.TransmissionTime
         this.V2ParkDatarate(distance_val)
         this.ComputeTransmissionTime(_task)
         this.AddLocalQueue(_task) # Task received in this.id local queue
         this.totalTskRecv = this.totalTskRecv + 1   
     elif _task.ParkFed == True:  # If Vv.id (RSU from the RSUManager list) in the "OffloadInParkFederation" is selected, Add or receive task task in Vv.id local queue
         this.LocalAvgTT = this.LocalAvgTT + this.TransmissionTime
         this.V2ParkDatarate(distance_val)
         this.ComputeTransmissionTime(_task)
         this.AddLocalQueue(_task) # Task received in this.id local queue
         this.totalTskRecv = this.totalTskRecv + 1   
 
    def ResultDelivery(this, _inExecTsk): # ParkManager class /
     if this.resultDel.qsize() > 0: # Maximum size of this.resultDel is 100 as defined, hold the results
       #print ("this.resultDel.qsize() Result deliver edge compute is:=",this.resultDel.qsize())
       _tt = this.resultDel.get()  # gets and holds the result for this task (Task in the execution delivery class)
       SrcVeh_object = Manager[_tt.SID] # Get source vehicle ID (Perhaps Object)from the Manger Class, from the complete list that describes 100 vehicles.
       dlv_dist = math.sqrt( ((this.x_axis - SrcVeh_object.x_axis)**2)+((this.y_axis - SrcVeh_object.y_axis)**2) )
       #print (" distance after execution is", dlv_dist)
       if dlv_dist > 0 and dlv_dist < 150:
         this.direct_delivery = this.direct_delivery + 1
         #print (" direct delivery called ", this.direct_delivery)
       elif dlv_dist >= 150 and dlv_dist < 300:
         this.oneHopDlv = this.oneHopDlv + 1
         #print (" 1 HP delivery called ", this.oneHopDlv)
       elif dlv_dist >= 300 and dlv_dist <= 450:
         this.twoHopDlv = this.twoHopDlv + 1
       elif dlv_dist >= 450 and dlv_dist < 600:
           this.threeHopDlv = this.threeHopDlv + 1  
       elif dlv_dist >= 600 and dlv_dist <= 750:
           this.fourHopDlv = this.fourHopDlv + 1             
       else:
         this.failure = this.failure + 1
       this.TaskSuccesfullyReturned(_inExecTsk) 
     else:
         #print("result delivery queue empty")
         this.resultDel.put(_inExecTsk) #Putting task(result) in the resultDel queue
    
    def TaskSuccesfullyReturned(this, _inExecTsk): 
     #print(this.id, "Executed Event ...!\n") 
      this.task_returnedPark = this.direct_delivery + this.oneHopDlv + this.twoHopDlv          
    
    def PrintStatPark(this): 
     print(this.id, " local compute ", this.countLocalExec, " Tsk Recv", this.totalTskRecv, " Pending Tsk ", this.que.qsize(), " Waiting Time = ", this.waitingTime, " Total execution time ", this.executionTime, " Task Federated ", this.taskFederated, " Federted task Recv ", this.FederatedTskReceived)
  
    
    #def PrintStatPark(this):
     # print(this.id, " Generated Tsk ", this.GenTsk , " local compute ", this.countLocalExec, " Offload = ", this.offloadTsk , " Tsk Recv", this.totalTskRecv, " Pending Tsk ", this.que.qsize(), " Waiting Time = ", this.waitingTime, " Total execution time ", this.executionTime)
    

class VehInfo: # Class is used during vehicle selection process based on distance or waiting time, add more selection features which will be available in main class automatically
    #def __init__(this,distance, wTime, wTimeV2V):
    def __init__(this,distance, wTime):
     this.distance = distance
     this.wtme = wTime
     #this.wtmev2v = wTimeV2V
     
class VehDestInfo: # Class is used during vehicle selection process based on distance or waiting time, add more selection features which will be available in main class automatically
    #def __init__(this,distance, wTime, wTimeV2V):
    def __init__(this,distance):
     this.distance = distance
     #this.wtmev2v = wTimeV2V     
     
class RSUInfo: # Class is used during vehicle selection process based on distance or waiting time, add more selection features which will be available in main class automatically
    def __init__(this,distance, wTime):
     this.distance = distance
     this.wtme = wTime

class ParkInfo: # Class is used during vehicle selection process based on distance or waiting time, add more selection features which will be available in main class automatically
    def __init__(this,distance, wTime):
     this.distance = distance
     this.wtme = wTime	 


class Vehicle:
   
   
   def  __init__(this, id, range, cpu, x_axis, y_axis, cpuAvg, cpuList, Off_mode):
     this.id= id
     this.range = range
     this.cpu = cpu
     this.x_axis = x_axis
     this.y_axis = y_axis
     this.cpuAvg = cpuAvg
     this.Off_mode = Off_mode
  #   this.RSU_mode = Off_mode
   #  this.park_mode = Off_mode
     this.Collection= {}
     this.nearbyRSU = {}
     this.nearbyPark = {}
     this.distManager= {}
     this.cpuList = cpuList
    # timer = threading.Timer(2.0, this.gfg) 
    # timer.start() 
     this.execution = 0 # we will reduce this with ticks to show task execution
     this.countLocalExec = 0
     this.TotalcountLocalExec = 0
     this.localTskExecutionQueue = 0
     
     this.offloadTsk = 0
     this.GenTsk = 0
     this.totalTskRecv=0
     this.queV2V = queue.Queue(maxsize=60000) # V2V offloading
     this.que = queue.Queue(maxsize=60000) # This is to hold the tasks for execution
     this.resultDel = queue.Queue(maxsize=40000) # This is to hold result for delivery
     this.waitingTime = 0
     this.TransmissionTime = 0
     this.executionTime = 0
     this.executionTimeV2V = 0
     this.LocalAvgWT =0 # local node average waiting time
     this.LocalAvgTT = 0
     this.direct_delivery = 0
     this.oneHopDlv =0
     this.twoHopDlv =0
     this.threeHopDlv = 0
     this.fourHopDlv = 0
     this.failure = 0
     this.steps = 0.0 # time simulation
     this.offloadTskRSU = 0
     this.offloadTskPark = 0
     this.AvgWaitintTimeVehicle =0
     this.Total_recv = 0
     this.localTsk = 0
     this.AvgWaitTimeV2V = 0
     
     
   
     this.execution = 0      # execution time of current task
      
     this.transV2V = 0                       # transmission delay v2v
     this.transV2Park = 0                # transmission delay v2Park
     this.transV2RSU = 0                     # transmission delay v2RSU
      
     this.deadlineLocal = 0                                  # deadline for local execution
     this.deadlineV2V = 0      # deadline for v2v
     this.deadlineV2Park =  0   # deadline for v2Park
     this.deadlineV2RSU = 0     # deadline for v2RSU
     this.deadLine =0
     this.deadlineMeet =0
     this.noDeadlineMeet = 0 
     
     this.V2VdeadMeet = 0
     this.V2VNoDeadMeet = 0
     
     this.V2ParkNoDeadMeet = 0
     this.V2ParkDeadMeet = 0
     
     
     this.V2RSUDeadMeet = 0
     this.V2RSUNoDeadMeet = 0
     this.RSUHandoff = 0
     this.ParkHandoff = 0
     
     this.localPark =0
     this.localV2V =0
     this.localRSU=0
     this.TskRSUtoRSU = 0
     this.rsuTorsuExecution=0
     
     this.waitingTimeV2V = 0
     this.LocalAvgWTV2V = 0
     this.localTskV2V = 0
     this.countLocalExecV2V = 0
     
     this.fiveHopDlv =0
     
     this.rateV2V = 0
     #this.v_ShortdistID =0

    
   def SimulationTime(this, stp):
    this.steps = stp
   #Dataset(this, time, mips, sid, did, swtime, dwtime, range, _dist, dec)
   def BuildDataset(this, _tsk, _dist, _wtime,_wtimeN1,_wtimeN2,_transTime1, _transTime2, _transTime3, decisn):
    #speed = traci.vehicle.getSpeed(this.id) 
    speedSID = traci.vehicle.getSpeed(_tsk.SID) 
     
    if decisn == "Local Compute": # 0
     #(this, time, mips, sid, did, swtime, dwtime, range, _dist,                               dec, t_Gen, l_tasks, v2v_task, f_v2v, f_v2RSU, t_returnedRSU, t_returnedpark, avg_wtimeVeh, avg_wtimeRSU)
     #data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize, this.cpu,0, 0,0,0, _tsk.SID,_tsk.DID, this.waitingTime, 0.0, len(this.Collection), 0.0, this.LocalAvgWT, 0,0, decisn)
     data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize, this.cpu,0, 0,0,0, this.waitingTime, 0,_wtime,_wtimeN1,_wtimeN2, _transTime1,_transTime2,_transTime3,speedSID,0, len(this.Collection), 0.0, 0, decisn)
     DataHolder.append(data)  

                                                                                                                                                                                          
    elif decisn == "V2V": # 1    
     speedDID = traci.vehicle.getSpeed(_tsk.DID)    
     #print ("SID decision 1 to vehicle",_tsk.SID) 
     #data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize,this.cpu,this.rateV2V, 0,0,0,_tsk.SID,_tsk.DID, this.waitingTime, Manager[_tsk.DID].waitingTime, len(this.Collection),_dist, this.AvgWaitTimeV2V, 0,0, decisn)
     data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize,this.cpu,this.rateV2V, 0,0,0, this.waitingTime, Manager[_tsk.DID].waitingTime, Manager[_tsk.NID].waitingTime,_wtimeN1,_wtimeN2, Manager[_tsk.NID].TransmissionTime,_transTime2, _transTime3,speedSID,speedDID, len(this.Collection),_dist,0, decisn)
     DataHolder.append(data)
    elif decisn == "V2RSU": # 2 
     
     #data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize ,this.cpu, RSUManager[_tsk.DID].rateV2RSU, 0,0,0,_tsk.SID,_tsk.DID, this.waitingTime, RSUManager[_tsk.DID].waitingTime, len(this.nearbyRSU),_dist, 0, RSUManager[_tsk.DID].LocalAvgWT,RSUManager[_tsk.DID].AvgWaitintTimeRsuToRsu, decisn)
     data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize ,this.cpu, RSUManager[_tsk.DID].rateV2RSU, 0,0,0, this.waitingTime, RSUManager[_tsk.DID].waitingTime,_wtimeN1,RSUManager[_tsk.NRID].waitingTime,_wtimeN2,_transTime1, RSUManager[_tsk.NRID].TransmissionTime,_transTime3,speedSID,0, len(this.nearbyRSU),_dist, RSUManager[_tsk.DID].handoff, decisn)
     DataHolder.append(data)	 
    elif decisn == "V2V With parked": # 3
     #data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize,this.cpu, ParkManager[_tsk.DID].rateV2Park, 0,0,0,_tsk.SID,_tsk.DID, this.waitingTime, ParkManager[_tsk.DID].waitingTime, len(this.nearbyPark),_dist, ParkManager[_tsk.DID].LocalAvgWT, 0,0, decisn )
     data = Dataset(this.steps, _tsk.MIPS,_tsk.dataSize,this.cpu, ParkManager[_tsk.DID].rateV2Park, 0,0,0,this.waitingTime, ParkManager[_tsk.DID].waitingTime,_wtimeN1,_wtimeN2,ParkManager[_tsk.NPID].waitingTime,_transTime1,_transTime2, ParkManager[_tsk.NPID].TransmissionTime,speedSID, 0,len(this.nearbyPark),_dist, 0, decisn )
     DataHolder.append(data)
 







    #if decisn == "Local Compute": # 0
     #(this, time, mips, sid, did, swtime, dwtime, range, _dist,                               dec, t_Gen, l_tasks, v2v_task, f_v2v, f_v2RSU, t_returnedRSU, t_returnedpark, avg_wtimeVeh, avg_wtimeRSU)
     
    # data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime, 0.0, len(this.Collection), 0.0, decisn, this.GenTsk, this.countLocalExec, 0.0, 0.0, 0.0, 0.0, 0.0, this.LocalAvgWT, 0)
    # DataHolder.append(data)
   # elif decisn == "V2V": # 1
   #  print ("SID decision 1 to vehicle",_tsk.SID) 
   #  data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime, Manager[_tsk.DID].waitingTime, len(this.Collection),_dist, decisn, this.GenTsk, 0, this.offloadTsk, this.failure, 0, 0,0, this.AvgWaitTimeV2V, 0)
   #  DataHolder.append(data)
   # elif decisn == "V2RSU": # 2 
   #  data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime, RSUManager[_tsk.DID].waitingTime, len(this.nearbyRSU),_dist, decisn, 0, 0, 0, 0 , this.failure , RSUManager[_tsk.DID].task_returnedRSU, 0, 0, RSUManager[_tsk.DID].AvgWaitTimeRSU )
   #  DataHolder.append(data)	 
   # elif decisn == "V2V With parked": # 3
   #  data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime, ParkManager[_tsk.DID].waitingTime, len(this.nearbyPark),_dist, decisn, 0, 0, this.offloadTsk, this.failure, 0,  0 , ParkManager[_tsk.DID].task_returnedPark, ParkManager[_tsk.DID].AvgWaitTimeV2V, 0 )
   #  DataHolder.append(data)




 
	 #(this, time, mips, sid, did, swtime, dwtime, range, dec):
     #print ("Offload to build dataset  called!")
	 # time, mips, sid, did, swtime, dwtime, dec





     ''''
   #Dataset(this, time, mips, sid, did, swtime, dwtime, range, _dist, dec)
   def BuildDataset(this, _tsk, _dist, _wtime, decisn):
    if decisn == 0:
     data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime, 0.0, len(this.Collection), 0.0, decisn)
     DataHolder.append(data)
    elif decisn == 1:
     print ("SID decision 1 to vehicle",_tsk.SID) 
     data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime, Manager[_tsk.DID].waitingTime, len(this.Collection),_dist, decisn)
     DataHolder.append(data)
    elif decisn == 2:
     data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime, RSUManager[_tsk.DID].waitingTime, len(this.nearbyRSU),_dist, decisn)
     DataHolder.append(data)	 
    elif decisn == 3:
     data = Dataset(this.steps, _tsk.MIPS, _tsk.SID,_tsk.DID, this.waitingTime,ParkManager[_tsk.DID].waitingTime, len(this.nearbyPark),_dist, decisn)
     DataHolder.append(data)
       '''              
   
   def  ComputeWaitingTime(this, _tsk):
    # print("this.cpu", this.cpu)
    # print ("Waiting time in this case ", this.waitingTime)
     this.waitingTime = this.waitingTime + _tsk.MIPS/this.cpu
   #  print("Task execution time", _tsk.MIPS/this.cpu)
    # print ("Waiting time in this case ", this.waitingTime)
     
   def  ComputeTransmissionTime(this, _tsk):
     #print("rateV2Vcomm", this.rateV2V) 
     #print("_tsk.dataSize", _tsk.dataSize) 
     #print("delay",_tsk.dataSize/this.rateV2V )
     this.TransmissionTime = this.TransmissionTime + _tsk.dataSize/this.rateV2V
     #print("this.TransmissionTime", this.TransmissionTime) 
  # def  ComputeWaitingTimeV2V(this, _tsk):
 #    this.waitingTimeV2V = this.waitingTimeV2V + _tsk.MIPS/this.cpu
  
  # def  ComputeAverageWaitingTime(this, _tsk): #Average Waiting time of this vehicle
      #this.LocalAvgWT = this.LocalAvgWT + this.waitingTime     
    #  this.AvgWaitintTimeVehicle= this.AvgWaitintTimeVehicle + this.LocalAvgWT
     # this.Total_recv = this.Total_recv + this.totalTskRecv     
    #  this.AvgWaitTimeV2V =this.AvgWaitintTimeVehicle/(this.Total_recv+ 0.1)
	 

   def ParkSelectionDistanceBased(this, selected_Park):

     spark_dist =  this.nearbyPark[selected_Park].distance
     d = ParkDistance = {}
     ParkDistance.update( {selected_Park: spark_dist} ) 
     for key, value in this.nearbyPark.items():   
     
       select_distance = this.nearbyPark[key].distance 
       ParkDistance.update( {key : select_distance} )    
       selected_Park = min(d, key=d.get)
       select_distance = min(d.values())     
         
      # if value.distance < spark_dist:
         #selected_Park = key
       #  spark_dist = this.nearbyPark[selected_Park].distance

     return selected_Park    







   def RSUSelectionDistanceBased(this, selected_RSU):

     srsu_dist =  this.nearbyRSU[selected_RSU].distance
     d = RSUDistance = {}
     RSUDistance.update( {selected_RSU: srsu_dist} ) 
     for key, value in this.nearbyRSU.items():   
     
       select_distance = this.nearbyRSU[key].distance 
       RSUDistance.update( {key : select_distance} )    
       selected_RSU = min(d, key=d.get)
       select_distance = min(d.values())     
         
      # if value.distance < srsu_dist:
         #selected_RSU = key
       #  srsu_dist = this.nearbyRSU[selected_RSU].distance

     return selected_RSU    



 #  def VehicleSelectionWaitintTimeBased(this):
#     selected_vehicle = list(this.Collection)[0]
  #   print("this.select", selected_vehicle)
 #    sveh_wtime =  Manager[selected_vehicle].waitingTime
  #   for key, value in Manager.items():   
  #     if value.waitingTime < sveh_wtime: 
 #          selected_vehicle = key   
  #         sveh_wtime = Manager[selected_vehicle].waitingTime                    
 #    print("selected v is", selected_vehicle) 
 #    return selected_vehicle	     
     
 
   def VehicleSelectionWaitintTimeBased(this):
     selected_vehicle = list(this.Collection)[0]
     #selected_vehicle = this.id
     sveh_wtime =  Manager[selected_vehicle].waitingTime
     d = waitingTimeList = {}
     waitingTimeList.update( {selected_vehicle: sveh_wtime} ) 
     for key, value in this.Collection.items():  
       
       select_wtime = Manager[key].waitingTime 
       waitingTimeList.update( {key : select_wtime} )    
       selected_vehicle = min(d, key=d.get)
       select_wtime = min(d.values())
       if select_wtime < this.waitingTime: 
          this.Off_mode = True
          
       else: 
          this.Off_mode = False       
     #print("selected v is", selected_vehicle) 
     return selected_vehicle	
 
 

 
   def VehicleSelectionDistanceBased(this):
     selected_vehicle = list(this.Collection)[0]   # gives the first key
     #import random
     #selected_vehicle = random.choice(list(this.Collection))
     sveh_dist =  this.Collection[selected_vehicle].distance
     for key, value in this.Collection.items():   
       if value.distance < sveh_dist:
         selected_vehicle = key
         sveh_dist = this.Collection[selected_vehicle].distance

     return selected_vehicle
	  
   def  PositionUpdate(this, x, y):
     this.x_axis = x
     this.y_axis = y
     #print ("Position Updated called", this.id, this.x_axis, this.y_axis)	 
	     

   def WithInRange_RSU(this): #Distance of that particular vehicle which wants to offload tasks is calculated from all 6 RSUs one by one.(val.id = One RSU)
    # print("Size of Manager_RSY: ", len(RSUManager))
     
     for key, val in RSUManager.items():
      # print(val.id, this.x_axis)
       distance = math.sqrt( ((this.x_axis - val.x_axis)**2)+((this.y_axis - val.y_axis)**2) )
       #print("RSURange", distance)
       #if distance < 70: Original Sir Asad Code, Check it
       #if distance < 80:   
       #if distance != 0 and distance < 80 and this.id != val.id:
       #if distance != 0 and distance < 20 and this.id != val.id:
       #if distance != 0 and distance < 9 and this.id != val.id:
       if distance != 0 and distance < 65 and this.id != val.id:
         #print ("Distance of vehicle this. id from RSU is", this.id, distance, this.range) # Note: this.id is "vehicle" and val.id is "RSU"
         #print("This.id, This. range, this.distance",this.id, this.range, distance)
         rsu = RSUInfo (distance, val.waitingTime)
         this.nearbyRSU[val.id] = rsu
         #print ("near is", rsu, val.id)

         


   def WithInRange_Park(this): #Distance of that particular vehicle which wants to offload tasks is calculated from all 6 parked vehicles one by one.(val.id = One park vehicle)
     #print("Size of Manager_Parl ", len(this.nearbyPark))
     for key, val in ParkManager.items():
       #print(val.id, this.x_axis)
       distance = math.sqrt( ((this.x_axis - val.x_axis)**2)+((this.y_axis - val.y_axis)**2) )
       #if distance < 70: Original Sir Asad Code, Check it
       if distance != 0 and distance < 65 and this.id != val.id:   
         #print ("Distance of vehicle this. id from RSU is", this.id, distance, this.range) # Note: this.id is "vehicle" and val.id is "RSU"
         #print("This.id, This. range, this.distance",this.id, this.range, distance)
         park = ParkInfo (distance, val.waitingTime)
         this.nearbyPark[val.id] = park
        # print ("near is", park, val.id)    

         
        
   def WithInRange(this):# Vehicle Class
     #print("Size of Manager: ", len(Manager))
     #this.v_ShortdistID =0
     for x, y in Manager.items():
       #print(x,  y.id, this.x_axis)
       distance = math.sqrt(((this.x_axis - y.x_axis)**2)+((this.y_axis - y.y_axis)**2))
       #if distance != 0 and distance < 60 and this.id != y.id:
       #if distance != 0 and distance < 11 and this.id != y.id:
       #if distance != 0 and distance < 44 and this.id != y.id: #less residence time
       dv = VehDestInfo(distance)
       this.distManager[y.id] = dv
       #print("The range is ", this.distManager )
       if distance != 0 and distance < 59 and this.id != y.id:
       #if distance != 0 and distance < 57 and this.id != y.id:
         #vv = VehInfo (distance, y.waitingTime, y.waitingTimeV2V)
         vv = VehInfo (distance, y.waitingTime)
         this.Collection[y.id] = vv
         #print ("This collection in this case ", this.Collection) # Complete dictionary with all the element(Key, Value)         
         #gg= {321:4,322:3,320:6,323:2}

          
  
   def GetRSUWaitingTime(this):
    rsize = len(this.nearbyRSU)
    if rsize < 0:
        return 9999
    for key, val in this.nearbyRSU.items():
        print(str(this.steps) + " RSU Waiting Time =" + str(val.wtme))
        return val.wtme
    return 9999    
    
   def GetRSUFogVehicleWaitingTime(this):
    vsize = len(this.Collection)
    if vsize < 0:
        return 9999
    for key, val in this.Collection.items():
        #print(str(this.steps) + " Top Vehicle Waiting Time =" + str(val.wtme) + ", V2V waiting time =" + str(val.wtmev2v))
        #return (val.wtme + val.wtmev2v)
        print(str(this.steps) + " Top Vehicle Waiting Time =" + str(val.wtme))
        return val.wtme 
      
    return 9999        
    
   def GetParkWaitingTime(this):
    psize = len(this.nearbyPark)
    if psize < 0:
        return 9999
    for key, val in this.nearbyPark.items():
        print(str(this.steps) + " Top Vehicle Waiting Time =" + str(val.wtme))
        return val.wtme
    return 9999       
    
    
   def OffloadToVehRSU(this, _task):
    
    #if this.waitingTime > 2.5:

     this.WithInRange()
     this.WithInRange_RSU()
     this.WithInRange_Park()

     rsize = len(this.nearbyRSU)
     psize = len(this.nearbyPark)
     vsize = len(this.Collection)
    
     RSUExecution =0
     ParkExecution =0
     VehExecution = 0
    
    
     destV = 0
     destR = 0
     destP = 0
    
     sveh_dist = 0
     srsu_dist = 0
     pveh_dist = 0
    
     import random 
  
     if rsize != 0:   # if rsu within range
             destR = this.RSUSelectionDistanceBased(list(this.nearbyRSU)[0])
             if RSUManager[destR].waitingTime < this.waitingTime:
             #destR = list(this.nearbyRSU)[0]
              #destR = random.choice(list(this.nearbyRSU))     
              _task.DID = destR
              DestRSU_object = RSUManager[destR]
             #RSUExecution = _task.MIPS/(DestRSU_object.cpu) 
              srsu_dist =  this.nearbyRSU[destR].distance   
              DestRSU_object.V2RSUDatarate(srsu_dist)
              RSUExecution = _task.MIPS/(DestRSU_object.cpu) + _task.dataSize/DestRSU_object.rateV2RSU
             else:   
                rsize = 0    
                #_task.localComp = True
                #this.AddLocalQueue(_task)
                #this.Off_mode = False                



     if psize !=0:
    
            destP = list(this.nearbyPark)[0]        
            if ParkManager[destP].waitingTime < this.waitingTime:
             #destP = random.choice(list(this.nearbyPark))      
             _task.DID = destP
             DestPark_object =ParkManager[destP]
            #ParkExecution = _task.MIPS/(DestPark_object.cpu)   
             pveh_dist =  this.nearbyPark[destP].distance # distManager list that contains the distance o  
             DestPark_object.V2ParkDatarate(pveh_dist)        
             ParkExecution = _task.MIPS/(DestPark_object.cpu) + _task.dataSize/DestPark_object.rateV2Park     
            #print("pveh_dist", pveh_dist)     

            else:   
              psize = 0  
              #_task.localComp = True
              #this.AddLocalQueue(_task)
              #this.Off_mode = False              

         
     if vsize != 0:
        
             destV = this.VehicleSelectionWaitintTimeBased()             
             _task.DID = destV    
             DestVeh_object = Manager[destV]             
             if destV!= this.id or DestVeh_object.Off_mode == False: 
              #VehExecution = _task.MIPS/(DestVeh_object.cpu)   
              #sveh_dist =  this.Collection[destV].distance # distManager list that contains the distance o  
              sveh_dist =  this.distManager[destV].distance # distManager list that contains the distance o
              #print("sveh_dist",sveh_dist)
              this.V2VDatarate(sveh_dist)
             #print("this.rateV2V offloed veh",this.rateV2V)
              VehExecution = _task.MIPS/(DestVeh_object.cpu) + _task.dataSize/(this.rateV2V) 
             else: 
                vsize = 0  
                #_task.localComp = True
               # this.AddLocalQueue(_task)
                #this.Off_mode = False
                
            
     #vval = 0
     #rval = 0
     #pval = 0
     
    # if  rsize != 0:
     rval = this.GetRSUWaitingTime()
      #rval = RSUManager[destR].waitingTime
     

     
    # if  vsize != 0: 
      #vval = Manager[destV].waitingTime
      #print("waiting time..", rval)
     vval = this.GetRSUFogVehicleWaitingTime()
     
    # if  psize != 0:
     pval = this.GetParkWaitingTime()
     #pval = ParkManager[destP].waitingTime
     
     
     lval = Manager[this.id].waitingTime # lcoal vehicle waiting time
    
        #lval1 = Manager[this.id].waitingTime # lcoal vehicle waiting time
        #oval = Manager[this.id].waitingTimeV2V # Offloading vehicle waiting time
        #lval = lval1 + oval # lcoal and offloading waiting time
        # print (" Local waiting time = " + str(lval))
        #print("RSUExec", RSUExecution)
        #print("destV", destV)
        

     if rsize == 0 and vsize != 0 and psize ==0:
        if vval < lval:# and VehExecution < this.deadLine:
         #   print("1. Offload to VEH v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
            this.OffloadToVeh(_task, vval, rval, pval, sveh_dist, destV, DestVeh_object)
        elif vval >= lval:
           # print("2. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
              _task.localComp = True
              this.AddLocalQueue(_task)
              #this.Off_mode = False
              #this.countLocalExec = this.countLocalExec + 1
                
     elif vsize == 0 and rsize !=0 and psize == 0: 
        if rval < lval:# and RSUExecution < this.deadLine:
           # print("3. Offload to RSU, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
            this.OffloadToRSU(_task, vval, rval,srsu_dist, pval, destR)
        elif rval >= lval:
          #  print("4. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
              _task.localComp = True
              this.AddLocalQueue(_task)
              #this.Off_mode = False
              #this.countLocalExec = this.countLocalExec + 1  
            
     elif rsize == 0 and vsize == 0 and psize != 0 : 
        if pval < lval:# and ParkExecution < this.deadLine:
         #   print("3. Offload to park, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "park wt=" + str(pval))
            this.OffloadToPark(_task, vval, rval, pval, pveh_dist, destP)
        elif pval >= lval:
           # print("4. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "park wt=" + str(pval))
              _task.localComp = True
              this.AddLocalQueue(_task)
              #this.Off_mode = False
              #this.countLocalExec = this.countLocalExec + 1              
            
        
     elif vsize !=0 and rsize !=0 and psize == 0:
        if rval < vval:
            if rval < lval:# and RSUExecution <= this.deadLine:
                #print("5. Offload to RSU, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
                this.OffloadToRSU(_task, vval, rval, srsu_dist, pval, destR)
            elif rval >= lval:
               # print("6. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
                _task.localComp = True
                this.AddLocalQueue(_task)
                #this.Off_mode = False
                #this.countLocalExec = this.countLocalExec + 1
        elif vval < rval :#and VehExecution < this.deadLine:
            if vval < lval:     #and sveh_dist!=0:
                print("7. Offload to Remote Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
                this.OffloadToVeh(_task, vval, rval, pval, sveh_dist, destV, DestVeh_object)
            elif vval >= lval:
                #print("8. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1
                
     
##########################################
     elif vsize != 0 and rsize == 0 and psize != 0:
        if vval < pval:
            if vval < lval :#and VehExecution < this.deadLine:
                #print("7. Offload to Remote Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
                this.OffloadToVeh(_task, vval, rval, pval, sveh_dist, destV, DestVeh_object)
            elif vval >= lval:
                #print("6. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1
        elif pval < vval:
            if pval < lval :#and ParkExecution <= this.deadLine:
                #print("3. Offload to park, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "park wt=" + str(pval))
                this.OffloadToPark(_task, vval, rval, pval, pveh_dist, destP)
            elif pval >= lval:
                #print("8. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1         
                                

     elif vsize == 0 and rsize != 0 and psize != 0:
        if rval < pval:
            if rval < lval :#and RSUExecution <= this.deadLine:
                #print("5. Offload to RSU, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
                this.OffloadToRSU(_task, vval, rval, srsu_dist, pval, destR)
            elif rval >= lval:
                #print("6. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1
        elif pval < rval :#and ParkExecution <= this.deadLine:
            if pval < lval:
               # print("3. Offload to park, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "park wt=" + str(pval))
                this.OffloadToPark(_task, vval, rval, pval, pveh_dist, destP)      
            elif pval >= lval:
                #print("8. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1                  
                
              
     elif vsize != 0 and rsize != 0 and psize != 0:
        if rval < pval and rval < vval:
            if rval < lval :#and RSUExecution <= this.deadLine:
                #print("5. Offload to RSU, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
                this.OffloadToRSU(_task, vval, rval, srsu_dist, pval, destR)
            elif rval >= lval:
                #print("6. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1
        elif pval < rval and pval < vval:
            if pval < lval :#and ParkExecution <= this.deadLine:
                #print("3. Offload to park, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "park wt=" + str(pval))
                this.OffloadToPark(_task, vval, rval, pval, pveh_dist, destP)
            elif pval >= lval:
                #print("8. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1                         
                
        elif vval < rval and vval < pval:
            if vval < lval :#and VehExecution < this.deadLine:
                #print("7. Offload to Remote Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
                this.OffloadToVeh(_task, vval, rval, pval, sveh_dist, destV, DestVeh_object)
            elif vval >= lval:
                #print("8. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))
               _task.localComp = True
               this.AddLocalQueue(_task)
               #this.Off_mode = False
               #this.countLocalExec = this.countLocalExec + 1  
                                
     else:
          _task.localComp = True
          this.AddLocalQueue(_task)
          #this.Off_mode = True #check it will be true or false
          #this.countLocalExec = this.countLocalExec + 1  
          print("9. Offload to Local Veh, remote v wt= " + str(vval) + " Local veh ="+str(lval) +  "rsu wt=" + str(rval))    

   # else:
    #    _task.localComp = True   
    #    this.AddLocalQueue(_task)
        #this.countLocalExec = this.countLocalExec + 1 
        #this.Off_mode = False
        
    


    # This is the part of "Environment aware multilevel computing architecture". Here we are taking the offloading decision. Improve this decision making.(See how we can add more parameters)     
   def OffloadTask(this,_task):
     #nearbyRSU then park and at the end offlod to vehicle gives optimal performace with less pending task, less average times(0.)
     
     #if len(this.nearbyRSU) > 0 and this.execution > this.deadlineLocal and this.deadlineLocal > this.deadlineV2V:
        #this.OffloadToRSU(_task)
        
        
     #elif  len(this.nearbyPark) > 0 and this.execution > this.deadlineLocal and this.deadlineLocal > this.deadlineV2V and this.deadlineV2V > this.deadlineV2RSU:
        #this.OffloadToPark(_task)     
     
     #elif  len(this.Collection) > 0 this.execution > this.deadlineLocal:
        #this.OffloadToVeh(_task)    
   
     #if len(this.nearbyRSU) > 0: 
         
      # this.OffloadToRSU(_task)
                
     
     #elif len(this.nearbyPark) > 0: 

       #this.OffloadToPark(_task)  
    
     
     
     if len(this.Collection) > 0:
      
       this.OffloadToVeh(_task)   

                
        
     else:
        this.AddLocalQueue(_task)
        this.countLocalExec = this.countLocalExec + 1
         
   #def OffloadToRSU(this, _task):
   def OffloadToRSU(this, _task, v_val, r_val, dist_val, p_val, dest):       

      DestRSU_object = RSUManager[dest]
     # RSUExecution = _task.MIPS/(DestRSU_object.cpu)
     #if this.execution > this.deadLine and this.deadLine > this.VehExecution and this.deadLine <= RSUExecution:
      #if RSUExecution <= this.deadLine:
      # or first RSU selected 
      _task.DID = dest
      DestRSU_object.RecvTask(_task, dist_val)
      this.offloadTskRSU = this.offloadTskRSU + 1
      RSUManager[dest] = DestRSU_object  
      obj = this.nearbyRSU[dest]
  
# This helps in dataset Generation 
  
      if this.nearbyRSU!={}:            
          RID_Shortdist = this.RSUSelectionDistanceBased(list(this.nearbyRSU)[0])
          #print("R_ID", RID_Shortdist)
          _task.NRID = RID_Shortdist
          DestNRSU_object = RSUManager[RID_Shortdist]
          #short_RSUdist =  this.nearbyRSU[_task.NRID].distance
          #print("distance RSU \n", short_RSUdist)
          #shortRSU_wtime =  RSUManager[_task.NRID].waitingTime
         # print("Waiting Time RSU \n", shortRSU_wtime) 


      if this.Collection!={}: 
          v_ShortdistID = this.VehicleSelectionDistanceBased()
          #print("VID", v_ShortdistID)
          _task.NID = v_ShortdistID
          DestNVeh_object = Manager[v_ShortdistID]
          #short_Vehdist =  this.Collection[_task.NID].distance
          #dist_V = min(gg.values())
          #print("distance \n", short_Vehdist)
          short_wtimeVeh =  Manager[_task.NID].waitingTime
          short_TtimeVeh =  Manager[_task.NID].TransmissionTime
          #print("Waiting Time \n", short_wtime)
          
      else:    
          short_wtimeVeh = 0 
          short_TtimeVeh = 0
          
      
      if this.nearbyPark!={}: 
          NPID_Shortdist = this.ParkSelectionDistanceBased(list(this.nearbyPark)[0])
          #print("R_ID", RID_Shortdist)
          _task.NPID = NPID_Shortdist
          DestNPark_object = ParkManager[NPID_Shortdist]
          short_wtimePark =  ParkManager[_task.NPID].waitingTime 
          short_TtimePark =  ParkManager[_task.NPID].TransmissionTime        
      else:           
          short_wtimePark = 0 
          short_TtimePark = 0          

      
      #objN = this.nearbyRSU[RID_Shortdist]

      this.BuildDataset(_task, obj.distance, obj.wtme, short_wtimeVeh, short_wtimePark, short_TtimeVeh, short_TtimePark,0, "V2RSU") # 2
      this.V2RSUDeadMeet = this.V2RSUDeadMeet + 1
      #print ("Task send from ", this.id, " to RSU ", dest)
        
        
   #  else:
   #     this.AddLocalQueue(_task)		
        
#   def OffloadToPark(this, _task):
   def OffloadToPark(this, _task, v_val, r_val, p_val, dist_val, destP):   
     #size = len(this.nearbyPark)
     #if size > 0:
     # dest = list(this.nearbyPark)[0]
      _task.DID = destP
      DestPark_object =ParkManager[destP]
     # ParkExecution = _task.MIPS/(DestPark_object.cpu)  
      
     #if this.execution > this.deadLine and this.deadLine > this.VehExecution and this.VehExecution > this.RSUExecution:      
      #if ParkExecution <= this.deadLine:      
      DestPark_object.RecvTask(_task, dist_val)
      this.offloadTskPark = this.offloadTskPark + 1
      ParkManager[destP] = DestPark_object  
      obj = this.nearbyPark[destP]
      


      if this.nearbyPark!={}: 
          NPID_Shortdist = this.ParkSelectionDistanceBased(list(this.nearbyPark)[0])
          #print("R_ID", RID_Shortdist)
          _task.NPID = NPID_Shortdist
          DestNPark_object = ParkManager[NPID_Shortdist]
          #short_wtimePark =  ParkManager[_task.NPID].waitingTime            
   
      
      if this.nearbyRSU!={}: 
          RID_Shortdist = this.RSUSelectionDistanceBased(list(this.nearbyRSU)[0])
          #print("R_ID", RID_Shortdist)
          _task.NRID = RID_Shortdist
          DestNRSU_object = RSUManager[RID_Shortdist]
          #short_RSUdist =  this.nearbyRSU[_task.NRID].distance
          #print("distance RSU \n", short_RSUdist)
          short_wtimeRSU =  RSUManager[_task.NRID].waitingTime
          short_TtimeRSU =  RSUManager[_task.NRID].TransmissionTime
         # print("Waiting Time RSU \n", shortRSU_wtime) 

      else:           
          short_wtimeRSU = 0 
          short_TtimeRSU = 0

      if this.Collection!={}: 
          v_ShortdistID = this.VehicleSelectionDistanceBased()
          #print("VID", v_ShortdistID)
          _task.NID = v_ShortdistID
          DestNVeh_object = Manager[v_ShortdistID]
          #short_Vehdist =  this.Collection[_task.NID].distance
          #dist_V = min(gg.values())
          #print("distance \n", short_Vehdist)
          short_wtimeVeh =  Manager[_task.NID].waitingTime
          short_TtimeVeh =  Manager[_task.NID].TransmissionTime
          #print("Waiting Time \n", short_wtime)
          
      else:    
          short_wtimeVeh = 0 
          short_TtimeVeh = 0
                     
      this.BuildDataset(_task, obj.distance, obj.wtme,short_wtimeVeh, short_wtimeRSU, short_TtimeVeh, short_TtimeRSU,0, "V2V With parked") #3 
      this.V2ParkDeadMeet = this.V2ParkDeadMeet + 1
      #print ("Task send from ", this.id, " to Parked ", destP)
 

 
     # else:
        #this.V2ParkNoDeadMeet = this.V2ParkNoDeadMeet + 1   
       # this.AddLocalQueue(_task)	
       # this.localPark = this.localPark + 1          
     #else:
       # this.AddLocalQueue(_task)	        
     
  # def OffloadToVeh(this, _task): # No execution only offloding, task is executed in the next iteration in ExecuteTask function and result is delivered in that function
   
   def OffloadToVeh(this, _task, v_val, r_val, p_val, dist_val, destV, DestVeh_object):
      size = len(this.Collection)
      if size > 0:
      #dest=list(this.Collection)[0]
       #dest = this.VehicleSelectionWaitintTimeBased()
       #dest = this.VehicleSelectionDistanceBased() # or VehicleSelectionWaitintTimeBased()
        #print("destV in over", destV)
       #dest = this.VehicleSelectionWaitintTimeBased()

     #  VehExecution = _task.MIPS/(DestVeh_object.cpu)            
     #if this.execution > VehExecution and VehExecution <= this.deadLine:  
      # if VehExecution <= this.deadLine: #and DestVeh_object.waitingTime < 4:    
        _task.DID = destV      
        DestVeh_object.RecvTask(_task, dist_val)
        this.offloadTsk = this.offloadTsk + 1

        if this.Collection!={}: 
          v_ShortdistID = this.VehicleSelectionDistanceBased()
          #print("VID", v_ShortdistID)
          _task.NID = v_ShortdistID
          DestNVeh_object = Manager[v_ShortdistID]
          #short_Vehdist =  this.Collection[_task.NID].distance
          #dist_V = min(gg.values())
          #print("distance \n", short_Vehdist)
          #short_wtime =  Manager[_task.NID].waitingTime
          #print("Waiting Time \n", short_wtime)


        if this.nearbyRSU!={}: 
          RID_Shortdist = this.RSUSelectionDistanceBased(list(this.nearbyRSU)[0])
          #print("R_ID", RID_Shortdist)
          _task.NRID = RID_Shortdist
          DestNRSU_object = RSUManager[RID_Shortdist]
          #short_RSUdist =  this.nearbyRSU[_task.NRID].distance
          #print("distance RSU \n", short_RSUdist)
          shortRSU_wtime =  RSUManager[_task.NRID].waitingTime
          shortRSU_Ttime =  RSUManager[_task.NRID].TransmissionTime
         # print("Waiting Time RSU \n", shortRSU_wtime) 

        else:
          shortRSU_wtime = 0
          shortRSU_Ttime = 0

        if this.nearbyPark!={}: 
          NPID_Shortdist = this.ParkSelectionDistanceBased(list(this.nearbyPark)[0])
          #print("R_ID", RID_Shortdist)
          _task.NPID = NPID_Shortdist
          DestNPark_object = ParkManager[NPID_Shortdist]
          short_wtimePark =  ParkManager[_task.NPID].waitingTime   
          short_TtimePark =  ParkManager[_task.NPID].TransmissionTime           
        else:           
          short_wtimePark = 0  
          short_TtimePark = 0
         
        obj = this.Collection[destV]
        #obj2 = this.Collection[v_ShortdistID]
        this.BuildDataset(_task, obj.distance, obj.wtme, shortRSU_wtime, short_wtimePark,shortRSU_Ttime,short_TtimePark,0,"V2V") # 1
          #this.BuildDataset(_task, 0, DestVeh_object.wtme ,"V2V") # 1
        this.V2VdeadMeet = this.V2VdeadMeet + 1 
        #print ("Task send from ", this.id, " to ", destV)
 
         
    #  else:
    #    this.AddLocalQueue(_task)		
	
   def AddLocalQueue(this, _task):
      this.LocalAvgWT = this.LocalAvgWT + this.waitingTime
      this.que.put(_task)
      this.ComputeWaitingTime(_task)
      this.localTskExecutionQueue = this.localTskExecutionQueue + 1
      #this.ComputeAverageWaitingTime(_tsk)
      #this.localTsk = this.localTsk + 1     
      

      if this.Collection!={}: 
          v_ShortdistID = this.VehicleSelectionDistanceBased()
          #print("VID", v_ShortdistID)
          _task.NID = v_ShortdistID
          DestNVeh_object = Manager[v_ShortdistID]
          #short_Vehdist =  this.Collection[_task.NID].distance
          #dist_V = min(gg.values())
          #print("distance \n", short_Vehdist)
          shortV_wtime =  Manager[_task.NID].waitingTime
          shortV_Ttime =  Manager[_task.NID].TransmissionTime
          #print("Waiting Time \n", short_wtime)

      else:
          shortV_wtime = 0 
          shortV_Ttime = 0

      if this.nearbyRSU!={}: 
          RID_Shortdist = this.RSUSelectionDistanceBased(list(this.nearbyRSU)[0])
          #print("R_ID", RID_Shortdist)
          _task.NRID = RID_Shortdist
          DestNRSU_object = RSUManager[RID_Shortdist]
          #short_RSUdist =  this.nearbyRSU[_task.NRID].distance
          #print("distance RSU \n", short_RSUdist)
          shortRSU_wtime =  RSUManager[_task.NRID].waitingTime
          shortRSU_Ttime =  RSUManager[_task.NRID].TransmissionTime
         # print("Waiting Time RSU \n", shortRSU_wtime) 

      else:
          shortRSU_wtime = 0
          shortRSU_Ttime = 0

      if this.nearbyPark!={}: 
          NPID_Shortdist = this.ParkSelectionDistanceBased(list(this.nearbyPark)[0])
          #print("R_ID", RID_Shortdist)
          _task.NPID = NPID_Shortdist
          DestNPark_object = ParkManager[NPID_Shortdist]
          short_wtimePark =  ParkManager[_task.NPID].waitingTime  
          short_TtimePark =  ParkManager[_task.NPID].TransmissionTime         
      else:           
          short_wtimePark = 0 
          short_TtimePark = 0          
         

 
      
      this.BuildDataset(_task,shortV_wtime,shortRSU_wtime,short_wtimePark,shortV_Ttime,shortRSU_Ttime,short_TtimePark,0, "Local Compute")
     

        

     
  # def AddV2VQueue(this, _tsk):
  #    this.LocalAvgWTV2V = this.LocalAvgWTV2V + this.waitingTimeV2V
  #    this.queV2V.put(_tsk)
   #   this.ComputeWaitingTimeV2V(_tsk)
      #this.ComputeAverageWaitingTimeV2V(_tsk)
   #   this.localTskV2V = this.localTskV2V + 1
   #   this.BuildDataset(_tsk,0.0,0.0, "Local Compute")      
	
 
   def Offload_V1(this, _task): 
     size = len(this.Collection)
     if size > 0:   # if vehicle within range (Check this list or RSU list which one of them is not empty)
        dest = list(this.Collection)[0] # Because this is the newly added element for the specific vehicle, this newly added vehicle is close to the offloading vehicel
        _task.DID = dest
        #tk = Task(400, this.id, dest)
        DestVeh_object = Manager[dest]
        DestVeh_object.RecvTask(_task)
       # this.offloadTsk = this.offloadTsk + 1
        Manager[dest] = DestVeh_object  
        


        #else:
          #this.Collection[v_ShortdistID].distance = 0  
          # _task.NID =0         
        
  
        this.BuildDataset(_task, obj.distance, obj.wtme ,"V2V") # 1this.BuildDataset(_task, obj.distance, obj.wtme ,"V2V") # 1  
        #print ("Task send from vehicle ", this.id, " to vehicle ", dest)
     else:
        this.AddLocalQueue(_task)
		
   def V2VDatarate(this,distance_val):
   
     N =  pow(10, -3)#( in watt
     Pt = 1 # W(in watt) # pt is the transmission power of the vehicles.
     B= 1
    # B= 20 # result will be in MHZ
     f= 5.9 * pow(10, 9)
     c = 3* pow(10,8)
     lamb = c/f
     Gt = 1
     Gr = 1
     pi = float(3.14)
     #d= 20
     #Pr = Pt*Gt*Gr*(lamb/(4*pi*d)^2
     G = pow((lamb/(pow(4*pi*distance_val, 2))),2)
     #G = lamb/pow(4*pi*distance_val, 2)
     #print("distance", distance_val)
     #print("G", G)   
    # L= pow(10, -(63.3 + 17.7*math.log(distance_val,10))/10)
     L = 63.3 + 17.7*math.log(distance_val,10)
     #print("L", L)
     #this.rateV2V = B * math.log((1+(Pt*L*G)/(N)),2)
     this.rateV2V = B * math.log((1+(Pt*L)/(N)),2)
     #print("rateV2Vcomm", this.rateV2V) 
   
   
   def RecvTask(this,_task, distance_val):
     this.AddLocalQueue(_task)
     this.LocalAvgTT = this.LocalAvgTT + this.TransmissionTime
     this.V2VDatarate(distance_val)
     #print("this.rateV2V in recv task", this.rateV2V)
     this.ComputeTransmissionTime(_task)     
     _task.localComp = False
     #this.AddV2VQueue(_task)
     this.totalTskRecv = this.totalTskRecv + 1   
   #  this.countLocalExec = this.countLocalExec + 1
     
      
	  		
   def PrintRanges(this):
      for key, value in this.Collection.items():
       print(this.id, " distance with ", key, " is: ", value)
	 
   def PrintStat(this):
       print(this.id, " Generated Tsk ", this.GenTsk , " local compute ", this.countLocalExec, " Offload = ", this.offloadTsk , " Tsk Recv", this.totalTskRecv, " Pending Tsk ", this.que.qsize(), " Waiting Time = ", this.waitingTime, " Total execution time ", this.executionTime)
   
   def EventGenerator(this):
     #(this, mips, source_vid, dest_vid, datasize, rateV2V, rateV2Park, rateV2RSU, this.deadlineLocal, this.dedlineV2V, this.dedlineV2Park, this.dedlineV2RSU ,false=RSUFed,ParkFed)
     # import random

      import random    
      roll_count =0
      mipsSum =0  

      epsln = random.uniform(0.001,0.003)       
      #epsln = random.randint(1,3) 
      #MIPS = random.randint(10,115) 
      #MIPS = random.randint(77,134) #use this with siz RSUs and parked vehicles
      #MIPS = random.randint(45,151) #use this with siz RSUs and parked vehicles
      MIPS = random.randint(45,150) #use this with siz RSUs and parked vehicles

     # this.deadLine = MIPS/cpuAvg + 0.25 # for the new updates in the execute task function.
      this.deadLine = MIPS/cpuAvg + 0.53  #Used to generate all resulats:

      



      if MIPS >=40 and MIPS <=63:
        dataSize = 1 
 
      elif MIPS >=64 and MIPS <=87:
        dataSize = 2 
      
      elif MIPS >=88 and MIPS <=111:
        dataSize = 3
      
      elif MIPS >=112 and MIPS <=135:
        dataSize = 4  

      elif MIPS >=136 and MIPS <=159:
        dataSize = 5  

      elif MIPS >=160 and MIPS <=183:
        dataSize = 6  
       
      #tk = Task(MIPS, this.id, 0,this.deadLine,  False, True, True)    
      #tk = Task(MIPS, this.id, 0, this.deadLine, dataSize, False, True, True) When parktopark offload is off 
      tk = Task(MIPS, this.id, 0, this.deadLine, dataSize, False, False, True) 
     # tk.localComp = False
      this.GenTsk = this.GenTsk + 1
                   

      this.OffloadToVehRSU(tk)

      #  this.countLocalExec = this.countLocalExec + 1       		
        #print (this.id, " Self-processed....!")

     
   def ResultDelivery(this, _inExecTsk): # Vehicle claSS
     
     if this.resultDel.qsize() > 0: # We are checking the result delivery queue of "this vehicle(this.id)" if it has the result of any task offloaded to it so that so that it may send it back. 
                                       # If there is any it will find that tasks associated with this vehicle with the help of the object _tt. SID is the source vehicle ID, which tells that which vehicle has generated this task and offloaded it aswell. So 
       _tt = this.resultDel.get()  # remove tsk from queue, _tt is the object of the the class and it is associated with the offloaded Task. _tt.SID will give the source vehicle ID of the vehicle which generated and offloaded the task.
       if _tt.SID != this.id: #(_tt.SID= The offloading vehicle, this.ID= The destination to which task is offloaded.)                                                                                                                                                                                                                                                                                                                                                                                  
         SrcVeh_object = Manager[_tt.SID]
         #print ("Hello result delivery",_tt)
         #print ("Hello result delivery",_tt.SID )
         dlv_dist = math.sqrt(((this.x_axis - SrcVeh_object.x_axis)**2)+((this.y_axis - SrcVeh_object.y_axis)**2))     
         if dlv_dist > 0 and dlv_dist < 150:  
         
           this.direct_delivery = this.direct_delivery + 1
           #print (" direct delivery called ", this.direct_delivery)
         elif dlv_dist >= 150 and dlv_dist < 300:
           this.oneHopDlv = this.oneHopDlv + 1
           
           #print (" 1 HP delivery called ", this.oneHopDlv)
         elif dlv_dist >= 300 and dlv_dist < 450:
           this.twoHopDlv = this.twoHopDlv + 1
           
         elif dlv_dist >= 450 and dlv_dist <= 600:
           this.threeHopDlv = this.threeHopDlv + 1
         
         elif dlv_dist >= 600 and dlv_dist <= 750:
           this.fourHopDlv = this.fourHopDlv + 1   
         else:
           this.failure = this.failure + 1
     #this.resultDel.put(_inExecTsk)
     else:
       this.resultDel.put(_inExecTsk)
       

   def gfg(this): 
    print(this.id, "Executed Event ...!\n") 
    this.EventGenerator()
    timer = threading.Timer(5.0, this.gfg) 
    timer.start() 
    timer.cancel()
    
   
    #def __del__(self):
    #timer.stop()

   def ExecuteTask(this):
      if this.que.empty()!= True:
        tpk = this.que.get()
        this.execution = tpk.MIPS/this.cpu      # execution time of current task
        this.executionTime = this.executionTime + this.execution  # measure total execution time at each veh
        this.waitingTime = this.waitingTime - this.execution  
         
        
        #this.execution = round(tpk.MIPS/this.cpu)      # execution time of current task
# compute waiting time, subtratc current execution. This shows how much a vehicle waits for execute a tasks.
           # Don't use this use the countLocalExec in TaskOffload and EventGenerator function
          #this.ResultDelivery(tpk) 
        if tpk.localComp == False:
          # compute waiting time, subtratc current execution. This shows how much a vehicle waits for execute a tasks.
          this.countLocalExecV2V =  this.countLocalExecV2V + 1   # Don't use this use the countLocalExec in TaskOffload and EventGenerator function     
          this.resultDel.put(tpk)
          this.ResultDelivery(tpk)
          #print("this.countLocalExecV2V", this.countLocalExecV2V) 
          
        else:
          this.TotalcountLocalExec =  this.TotalcountLocalExec + 1    
        
        

		 
   def UpdateExecutionTimer(this):
     
     if this.execution >  0:
       this.execution = this.execution - 1           
     else : 
       this.ExecuteTask()



   # import random
    # epsln = random.uniform(0.5625,1.875)
     #if this.execution !=0 and this.execution > 1.4:
      #    this.execution = this.execution - epsln        
     #elif this.execution <= 1.4:      
      # this.ExecuteTask()  

     
   # if this.execution !=0:
   #  import time
   #  import asyncio
   #  async def do_some_work():
   #    this.ExecuteTask()
   #    print("Waiting ")
       #await asyncio.sleep(x)
   #  loop = asyncio.get_event_loop()
  #   loop.run_until_complete(do_some_work())   
  #  elif this.execution :
         

     
     #if this.waitingTime !=0:
      #this.waitingTime = this.waitingTime - 1
	  


# xloc, yloc, id, range, CPU
#edge1 = EdgeCompute(1711.03,2341.07,1, 40, 100)
#edge2 = EdgeCompute(2257.12,2273.51,2, 40, 100)
#edge3 = EdgeCompute(1656.19,2046.40,3, 40, 100)
#edge4 = EdgeCompute(1974.48,2363.03,4, 40, 100)



edge1 = EdgeCompute(198.15,702.22,1, 40, 250) # Different corners optimaly devided
edge2 = EdgeCompute(698.35,702.22,2, 40, 250)
edge3=  EdgeCompute(196.40,202,3, 40, 250)
edge4 = EdgeCompute(694.85,207.77,4, 40, 250)
edge5 = EdgeCompute(192.91,457.87,5, 40, 250)
edge6 = EdgeCompute(701.85,457.87,6, 40, 250)




RSUManager[1] = edge1
RSUManager[2] = edge2
RSUManager[3] = edge3
RSUManager[4] = edge4   
RSUManager[5] = edge5 
RSUManager[6] = edge6

#RSUManager[7] = edge7   
#RSUManager[8] = edge8 
#RSUManager[9] = edge9
#RSUManager[10] = edge10   
#RSUManager[11] = edge11 
#RSUManager[12] = edge12


#RSUManager[13] = edge13   
#RSUManager[14] = edge14 
#RSUManager[15] = edge15
#RSUManager[16] = edge16   
#RSUManager[17] = edge17 
#RSUManager[18] = edge18






#Park1 = P_Vehicle(294.11,804.36,1, 10, 100) #used to generated initial graphs
#Park2 = P_Vehicle(195.44,704.91,2, 10, 100)
#Park3 = P_Vehicle(203.72,606.35,3, 10, 100)
#Park4 = P_Vehicle(194.17,605.40,4, 10, 100)
#Park5 = P_Vehicle(103.43,794.41,5, 10, 100)
#Park6 = P_Vehicle(103.43,506.07,6, 10, 100)

Park1 = P_Vehicle(793.40,503.40,1, 10, 100) # At one location
Park2 = P_Vehicle(795.05,503.86,2, 10, 100)
Park3 = P_Vehicle(794.44,505.05,3, 10, 100)
Park4 = P_Vehicle(794.99,504.26,4, 10, 100)
Park5 = P_Vehicle(791.48,503.26,5, 10, 100)
Park6 = P_Vehicle(796.58,506.77,6, 10, 100)


#Park1 = P_Vehicle(408.32,803.52,1, 10, 50) # At 4 sides optimal devision
#Park2 = P_Vehicle(102.17,481.39,2, 10, 50)
#Park3 = P_Vehicle(792.59,485.64,3, 10, 50)
#Park4 = P_Vehicle(486.68,105.37,4, 10, 50)
#Park5 = P_Vehicle(392.66,605.68,4, 10, 50)
#Park6 = P_Vehicle(394.64,305.34,4, 10, 50)



ParkManager[1] = Park1
ParkManager[2] = Park2
ParkManager[3] = Park3
ParkManager[4] = Park4   
ParkManager[5] = Park5 
ParkManager[6] = Park6



step = 0
seed(1)  # generate some random numbers
while step < 1000:
   print ("Inside loop")
   traci.simulationStep()
   for key, val in RSUManager.items():
     RSUManager[key].UpdateExecutionTimer()   
   
   for key, val in ParkManager.items(): # Runs the number of times equal to Park vehicles, = 6 , Key is ParkV id and val is bbject of V_parked
     ParkManager[key].UpdateExecutionTimer()   
                                                                                                                                                                                            
   roll_count = 0  
   cpuSum = 0   
   for veh_id in traci.vehicle.getIDList():
        position = traci.vehicle.getPosition(veh_id)
        #print("position", position)
        #print ("id, psoition",veh_id, position)
        if veh_id in Manager:
           #print(" Vehicle already in manager", veh_id)
           Veh_object = Manager[veh_id]
           Veh_object.PositionUpdate(position[0], position[1])
           #print ("Position ", position[0], position[1])
           Veh_object.SimulationTime(step)         
           from random import random
           if random() > 0.05:
            Veh_object.EventGenerator()
             #print("Hello")
             #Veh_object.gfg()
             #print(" This is the vehicle object\n", Veh_object, veh_id)
		     
           Veh_object.UpdateExecutionTimer()
           
           Manager[veh_id] = Veh_object # Add vehicle object in manager
           
        else:
           import random
           
           if roll_count == 0:
               cpuList = {}
           
           roll_count += 1
           #print("ro_count", roll_count)
           #(this, id, range, cpu, x_axis, y_axis) # Vehicle Class
           #print( veh_id, "x_axis =", position[0], " y_axis=", position[1])
           

           #cpu = random.randint(130,145)
           #cpu = random.randint(90,100)
           #cpu = random.randint(80,100)
           
           cpu = random.randint(160,185)
           #cpu = random.randint(120,150) # this is used
           #cpu = random.randint(80,100) #(Graphs in collective intelligence paper taken on this reading)
           cpuList.update({veh_id : cpu})           
           cpuSum = cpuSum + cpu
           cpuAvg = cpuSum//roll_count
           keys = range(roll_count)
           #cpuList[veh_id] = cpu   
           #print("Cpu list", cpuList)
           #print("length is",len(cpuList))                             
           #print("Cpu avg inside loop", cpuAvg)
           Off_mode = 0
           v = Vehicle(veh_id, 10, cpu, position[0], position[1], cpuAvg, cpuList, Off_mode)
           #v = Vehicle(veh_id, 10, 100.0, position[0], position[1])
           v.PositionUpdate(position[0], position[1])
           v.SimulationTime(step)
           #ParsePosition(position)
           Manager[veh_id] = v # Add in the manager
           #print(" Add this in  manager", veh_id)
   #print("Final CPU average is",  cpuSum//roll_count) 
   step += 1
   #print("Step size\n\n\n", step)


   #print ("Departed vehicles", traci.simulation.getDepartedNumber())
   #print ("Loaded vehicles", traci.simulation.getLoadedNumber()) 
   #print ("Vehicles in simulation", traci.vehicle.getIDCount())
   #print ("ID present in network ", traci.vehicle.getIDList());
   

print("Print stat called ")
Total_offload=0
Total_pending=0
Total_recv=0
Total_gen=0
Total_compute=0
AvgWaitintTime =0
AvgTransmissionTime = 0
Ddelivery = 0
oneHopDel =0
twoHopDel = 0
threeHopDel = 0
fourHopDel = 0
fiveHopDel = 0


failureDel =0
total_deadMeetV2V = 0
total_nodeadMeetV2V = 0
total_local_V2V = 0 
Total_pendingV2V = 0
AvgWaitintTimeV2V = 0
Total_recvV2V = 0
Total_computeV2V = 0
Total_in_localQueue = 0 
Total_execution_time  = 0
countTskExecutionQueueVeh = 0





Total_offload_rsu_via_handoff = 0
Total_offload_rsu=0

Total_pending_rsu=0
Total_recv_rsu=0
Total_recv_  =0
Total_compute_rsu=0
AvgWaitintTime_rsu =0
AvgTransmissionTime_rsu = 0
Ddelivery_rsu = 0
oneHopDel_rsu =0
twoHopDel_rsu = 0
threeHopDel_rsu = 0
fourHopDel_rsu = 0
fiveHopDel_rsu = 0
failureDel_rsu =0
LocalExec = 0
total_deadMeetV2RSU = 0
total_nodeadMeetV2RSU = 0
total_local_V2RSU = 0 
V2VExec = 0
execution_TimeV2V=0
execution_TimeRSU = 0


total_Count_RSUtoRSUHandoff =0 
Total_offload_rsuTorsu=0
Total_pending_RSUtoRSU= 0
Total_execution_RSUtoRSU = 0
AvgWaitintTime_RSUtoRSU =0
AvgTransmissionTime_RSUtoRSU = 0
Total_recv_RSUtoRSU = 0
Total_Compute_RSUtoRSU = 0

Ddelivery_RSUtoRSU = 0
oneHopDel_RSUtoRSU = 0
twoHopDel_RSUtoRSU = 0
threeHopDel_RSUtoRSU = 0
fourHopDel_RSUtoRSU = 0
fiveHopDel_RSUtoRSU = 0
failureDel_RSUtoRSU = 0
execution_TimeRSUtoRSU = 0







Total_offload_Park=0
Total_pending_Park=0
Total_recv_Park=0
Total_recv_  =0
Total_compute_Park=0
AvgWaitintTime_Park =0
AvgTransmissionTime_Park = 0
Ddelivery_Park = 0
oneHopDel_Park =0
twoHopDel_Park = 0
threeHopDel_Park = 0
fourHopDel_Park = 0
failureDel_Park =0
LocalExec = 0
total_deadMeetV2Park = 0
total_nodeadMeetV2Park = 0
total_RSUtoParkHandoff = 0
total_local_V2Park = 0 
res_del = 0
execution_TimePark =0

     
for x, y in Manager.items():
 #print ("Calling print of ", x)
 y.PrintStat()
 Total_offload = Total_offload + y.offloadTsk
 Total_pending = Total_pending + y.que.qsize()
 Total_recv = Total_recv + y.totalTskRecv # Task received for V2V case
 #Total_in_localQueue = Total_in_localQueue + y.localTsk # cehck again. task in the local queue this will be used to calculate the waiting time by dividng it the total task executed
 Total_gen = Total_gen + y.GenTsk
 Total_compute = Total_compute + y.TotalcountLocalExec
 #Total_compute = Total_compute + y.countLocalExec
 AvgWaitintTime = AvgWaitintTime + y.LocalAvgWT
 AvgTransmissionTime = AvgTransmissionTime + y.LocalAvgTT
                              
 Ddelivery = Ddelivery + y.direct_delivery
 oneHopDel = oneHopDel + y.oneHopDlv
 twoHopDel = twoHopDel + y.twoHopDlv
 threeHopDel = threeHopDel + y.threeHopDlv
 fourHopDel = fourHopDel + y.fourHopDlv
 failureDel = failureDel + y.failure

 res_del = res_del + y.resultDel.qsize()
 Total_offload_rsu = Total_offload_rsu + y.offloadTskRSU


 Total_offload_Park = Total_offload_Park + y.offloadTskPark
 LocalExec = LocalExec + y.TotalcountLocalExec
 #LocalExec = LocalExec + y.countLocalExec
 total_deadMeetV2V = total_deadMeetV2V + y.V2VdeadMeet
 total_nodeadMeetV2V = total_nodeadMeetV2V + y.V2VNoDeadMeet
 total_local_V2V = total_local_V2V + y.countLocalExec
 total_deadMeetV2RSU= total_deadMeetV2RSU + y.V2RSUDeadMeet
 total_nodeadMeetV2RSU = total_nodeadMeetV2RSU + y.V2RSUNoDeadMeet
 total_deadMeetV2Park= total_deadMeetV2Park + y.V2ParkDeadMeet
 total_nodeadMeetV2Park = total_nodeadMeetV2Park + y.V2ParkNoDeadMeet
 #total_RSUtoParkHandoff = total_RSUtoParkHandoff + y.ParkHandoff 
 
 # For v2v Exec
 V2VExec = V2VExec + y.countLocalExecV2V
 Total_pendingV2V = Total_pendingV2V + y.queV2V.qsize() 
 AvgWaitintTimeV2V = AvgWaitintTimeV2V + y.LocalAvgWTV2V
 Total_recvV2V = Total_recvV2V + y.totalTskRecv
 Total_computeV2V = Total_computeV2V + y.countLocalExecV2V
 Total_execution_time = Total_execution_time + y.executionTime
 execution_TimeV2V = execution_TimeV2V + y.executionTime
 countTskExecutionQueueVeh = countTskExecutionQueueVeh + y.localTskExecutionQueue
 
 
 


for Kk, Vv in RSUManager.items():
   Vv.PrintStatRSU()
   
   
   Total_pending_rsu = Total_pending_rsu + Vv.que.qsize()
   Total_recv_rsu = Total_recv_rsu + Vv.totalTskRecv
   Total_compute_rsu = Total_compute_rsu + Vv.countLocalExec
   AvgWaitintTime_rsu = AvgWaitintTime_rsu + Vv.LocalAvgWT
   AvgTransmissionTime_rsu = AvgTransmissionTime_rsu + Vv.LocalAvgTT
   
   Ddelivery_rsu = Ddelivery_rsu + Vv.direct_delivery
   oneHopDel_rsu = oneHopDel_rsu + Vv.oneHopDlv
   twoHopDel_rsu = twoHopDel_rsu + Vv.twoHopDlv
   threeHopDel_rsu = threeHopDel_rsu + Vv.threeHopDlv
   fourHopDel_rsu = fourHopDel_rsu + Vv.fourHopDlv
   failureDel_rsu = failureDel_rsu + Vv.failure
   total_local_V2RSU = total_local_V2RSU + Vv.countLocalExec

  #v RSU to RSU
  
   Total_offload_rsu_via_handoff = Vv.offloadTskRSU + Total_offload_rsu_via_handoff 
    
   Total_offload_rsuTorsu = Total_offload_rsuTorsu + Vv.taskFederated   
   total_Count_RSUtoRSUHandoff = total_Count_RSUtoRSUHandoff + Vv.RSUHandoff
   Total_execution_RSUtoRSU = Total_execution_RSUtoRSU + Vv.countTotalExecRSUtoRSU
   AvgWaitintTime_RSUtoRSU = AvgWaitintTime_RSUtoRSU + Vv.LocalAvgWTRsuToRsu
   AvgTransmissionTime_RSUtoRSU = AvgTransmissionTime_RSUtoRSU + Vv.LocalAvgTTRsutoRsu
   Total_pending_RSUtoRSU = Total_pending_RSUtoRSU + Vv.queRSUtoRSU.qsize()
   Total_recv_RSUtoRSU = Total_recv_RSUtoRSU + Vv.totalTskRecvRSUtoRSU
   Total_Compute_RSUtoRSU = Total_Compute_RSUtoRSU + Vv.countTotalExecRSUtoRSU
   Ddelivery_RSUtoRSU = Ddelivery_RSUtoRSU + Vv.direct_deliveryRSUtoRSU
   oneHopDel_RSUtoRSU = oneHopDel_RSUtoRSU + Vv.oneHopDlvRSUtoRSU 
   twoHopDel_RSUtoRSU = twoHopDel_RSUtoRSU + Vv.twoHopDlvRSUtoRSU 
   threeHopDel_RSUtoRSU = threeHopDel_RSUtoRSU + Vv.threeHopDlvRSUtoRSU 
   fourHopDel_RSUtoRSU = fourHopDel_RSUtoRSU + Vv.fourHopDlvRSUtoRSU 
   failureDel_RSUtoRSU =   failureDel_RSUtoRSU + Vv.failureRSUtoRSU
   execution_TimeRSU = execution_TimeRSU + Vv.executionTime
   execution_TimeRSUtoRSU = execution_TimeRSUtoRSU + Vv.executionTimeRSUtoRSU
   
   


   
for Kk, Vv in ParkManager.items():
   Vv.PrintStatPark()
   
   Total_pending_Park = Total_pending_Park + Vv.que.qsize()
   Total_recv_Park = Total_recv_Park + Vv.totalTskRecv
   Total_compute_Park = Total_compute_Park + Vv.countLocalExec
   
   AvgWaitintTime_Park = AvgWaitintTime_Park + Vv.LocalAvgWT
   AvgTransmissionTime_Park = AvgTransmissionTime_Park + Vv.LocalAvgTT
   
   Ddelivery_Park = Ddelivery_Park + Vv.direct_delivery
   oneHopDel_Park = oneHopDel_Park + Vv.oneHopDlv
   twoHopDel_Park = twoHopDel_Park + Vv.twoHopDlv
   threeHopDel_Park = threeHopDel_Park + Vv.threeHopDlv
   threeHopDel_Park = threeHopDel_Park + Vv.threeHopDlv
   fourHopDel_Park = fourHopDel_Park + Vv.fourHopDlv
   failureDel_Park = failureDel_Park + Vv.failure
   total_local_V2Park = total_local_V2Park + Vv.countLocalExec
   execution_TimePark = execution_TimePark + Vv.executionTime



print ("************** V2V Stats ****************")
print ("Total offload to Vehicles =", Total_offload)
print ("Total locally execution for V2V  =",  total_local_V2V)

print ("Total locally executed =", LocalExec)
print ("Total offloaded  executed =", V2VExec)
print ("del queue =", res_del)
print ("Total Pending =", Total_pending)
print ("Total Recv =", Total_recv)
print ("Total Task Gen =", Total_gen)
#print ("Total_execution_time =", Total_execution_time/(LocalExec + V2VExec))

print ("Total Compute =", Total_compute)
try:
    print ("Avg waiting time = ",(AvgWaitintTime/(Total_recv)))
    print ("Avg Transmission time = ",(AvgTransmissionTime/(Total_offload)))
    print ("Efficiency ", (V2VExec + Total_compute) / Total_gen)
except ZeroDivisionError:
    print ("Avg waiting time = ",(AvgWaitintTime))
    print ("Avg Transmission time = ",(AvgTransmissionTime))
    print ("Efficiency ", V2VExec + Total_compute)
print ("Direct delivery ", Ddelivery)
print ("One hop delivery ", oneHopDel)
print ("Two hop delivery ", twoHopDel)
print ("Three hop delivery ", threeHopDel)
print ("Four hop delivery ", fourHopDel)
print ("Failure delivery ", failureDel)
print ("Deadline Meet ", total_deadMeetV2V)
print ("No deadline Meet ", total_nodeadMeetV2V)
print (" Vehilces ", len(Manager))
print ("************** *** ****************")

###########################################
print ("************** V2Park Stats ****************")
print("Total offload to Parked Vehicles =", Total_offload_Park)
print ("Total locally execution for V2Park  =",  total_local_V2Park)
print ("Total Pending =", Total_pending_Park) 
print ("Total Recv =", Total_recv_Park)
print ("Total Compute =", Total_compute_Park)
print ("Total Compute =", Total_compute)
try:
   print ("Avg waiting time = ", AvgWaitintTime_Park/(Total_recv_Park ))
   print ("Avg Transmission time = ", AvgTransmissionTime_Park/(Total_offload_Park ))
   print ("Efficiency ", Total_compute_Park/ (Total_recv_Park )) #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
except ZeroDivisionError:
   print ("Avg waiting time = ", AvgWaitintTime_Park)
   print ("Avg Transmission Time = ", AvgTransmissionTime_Park)
   print ("Efficiency ", Total_compute_Park) #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2  
#print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
print ("Direct delivery ", Ddelivery_Park)
print ("One hop delivery ", oneHopDel_Park)
print ("Two hop delivery ", twoHopDel_Park)
print ("Three hop delivery ", threeHopDel_Park)
print ("Failure delivery ", failureDel_Park)
print ("Deadline Meet ", total_deadMeetV2Park)
print ("No deadline Meet ", total_nodeadMeetV2Park)
print (" # of Parked Vehicles", len(ParkManager))
print ("************** *** ****************")

print ("************** *** ****************")
##########################################

print ("************** V2RSU Stats ****************")

print("Total offload to RSUs =", Total_offload_rsu + Total_offload_rsu_via_handoff)
print("Total offload to RSUs =", Total_offload_rsu)
print ("Total locally execution for V2RSU =",  total_local_V2RSU)
print ("Total Pending =", Total_pending_rsu) 
print ("Total Recv =", Total_recv_rsu)
print ("Total Compute =", Total_compute_rsu)
try:
    print ("Avg waiting time = ", AvgWaitintTime_rsu/(Total_recv_rsu ))  #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
    print ("Avg Transmission time = ", AvgTransmissionTime_rsu/(Total_offload_rsu ))
    print ("Efficiency ", Total_compute_rsu / (Total_recv_rsu )) #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2     
except ZeroDivisionError:
    print ("Avg waiting time = ", AvgWaitintTime_rsu)
    print ("Avg Transmission time = ", AvgTransmissionTime_rsu)
    print ("Efficiency ", Total_compute_rsu) #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
print ("Direct delivery ", Ddelivery_rsu)
print ("One hop delivery ", oneHopDel_rsu)
print ("Two hop delivery ", twoHopDel_rsu)
print ("Three hop delivery ", threeHopDel_rsu)
print ("Failure delivery ", failureDel_rsu)
print ("Deadline Meet ", total_deadMeetV2RSU)
print ("No deadline Meet ", total_nodeadMeetV2RSU)
print (" # of RSUs ", len(RSUManager))
print ("************** *** ****************")






print ("************** V2RSU to RSU Stats ****************")




print("Total event generated =", Total_gen)
print("offloaded =", Total_offload_rsuTorsu + total_Count_RSUtoRSUHandoff)
print ("Total event executed =", Total_execution_RSUtoRSU)
print ("Pending Event =", Total_pending_RSUtoRSU)
#print ("Total Compute =", Total_compute_rsu) Total_execution_RSUtoRSU cehck it
try:
   print ("Avg waiting time = ", AvgWaitintTime_RSUtoRSU/Total_recv_RSUtoRSU)  #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
   print ("Avg Transmission time = ", AvgTransmissionTime_RSUtoRSU/(Total_recv_RSUtoRSU))  
   print ("Total Efficiency ", Total_Compute_RSUtoRSU/Total_recv_RSUtoRSU)
except ZeroDivisionError:
   print ("Avg waiting time = ", AvgWaitintTime_RSUtoRSU)  #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
   print ("Avg Transmission time = ", AvgTransmissionTime_RSUtoRSU)  
   print ("Total Efficiency ", Total_Compute_RSUtoRSU)
print ("Good decisions ", Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU)
print ("Bad decisions ", failureDel_RSUtoRSU)
print ("One-hop delivery ", oneHopDel_RSUtoRSU)
print ("Two-hop delivery ", twoHopDel_RSUtoRSU)
print ("Three hop delivery ", threeHopDel_RSUtoRSU) #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
print ("Failure delivery ", failureDel_RSUtoRSU)
print ("Deadline Meet ", total_deadMeetV2RSU)
print ("No deadline Meet ", total_nodeadMeetV2RSU)


print (" # of RSUs ", len(RSUManager))
print ("************** *** ****************")








print ("************** TOTAL Stats V2V and V2RSU Stats ****************")

print ("Total Pending =", Total_pending_rsu + Total_pending +Total_pending_Park +Total_pending_RSUtoRSU)
print ("Total Recv @ Veh =", Total_recv)
print ("Total Recv @ RSU =", Total_recv_rsu)
print ("Total Recv @ Park =", Total_recv_Park)
print ("Total Recv @ RSUtoRSU=", Total_recv_RSUtoRSU)



print ("Total Compute =", Total_compute_rsu + Total_compute + Total_compute_Park)
print ("Total Compute V2V, RSU and RSU to RSU =", Total_compute_rsu + Total_compute + Total_compute_Park + Total_Compute_RSUtoRSU)
#print ("Avg waiting time = ", ((AvgWaitintTime_rsu/(Total_recv_rsu + 2)) + (AvgWaitintTime/(Total_recv + 2)))/2.0 ) #print ("Efficiency ", Total_compute_rsu / Total_recv_rsu) ZerooDivisionError: division by zero, that why we divided by 2
try:
   print ("Avg waiting time = ", ((AvgWaitintTime_rsu/(Total_recv_rsu )) + (AvgWaitintTime/(Total_recv )) + (AvgWaitintTime_Park/(Total_recv_Park)))/3.0 )
except ZeroDivisionError:
   print ("Avg waiting time = ", ((AvgWaitintTime_rsu) + (AvgWaitintTime) + (AvgWaitintTime_Park))/3.0 )
print ("Direct delivery ", Ddelivery_rsu + Ddelivery +Ddelivery_Park)
print ("One hop delivery ", oneHopDel_rsu + oneHopDel + oneHopDel_Park)
print ("Two hop delivery ", twoHopDel_rsu + twoHopDel + twoHopDel_Park)
print ("Three hop delivery ", threeHopDel_rsu + threeHopDel + threeHopDel_Park)
print ("Failure delivery ", failureDel_rsu + failureDel + failureDel_Park)
print ("Total Deadline Meet ", total_deadMeetV2V + total_deadMeetV2RSU + total_deadMeetV2Park)
print ("Total No deadline Meet ", total_nodeadMeetV2V + total_nodeadMeetV2RSU + total_nodeadMeetV2Park)
print ("Total RSU to RSU Handoff ", total_Count_RSUtoRSUHandoff)
print ("Total RSU to Park handoff ", total_RSUtoParkHandoff )
try:
   print ("Efficiency ", (Total_compute_rsu + Total_compute + Total_compute_Park) / Total_gen)
   print ("Efficiency total  ", (Total_compute_rsu + Total_compute + Total_compute_Park + Total_Compute_RSUtoRSU ) / Total_gen)
except ZeroDivisionError:
   print ("Efficiency ", (Total_compute_rsu + Total_compute + Total_compute_Park))
   print ("Efficiency total  ", (Total_compute_rsu + Total_compute + Total_compute_Park + Total_Compute_RSUtoRSU))
print (" # of RSUs ", len(RSUManager))
print (" # of Parked Vehicles ", len(ParkManager))
print (" # of Vehilces ", len(Manager))
print ("************** *** ****************")



traci.close()



f = open("data_set_new.csv","a")
i = 0
f.write("Tech")  
f.write(",") 
f.write("Total Vehiclular nodes")   
f.write(",") 
f.write("Total event generated")
f.write(",") 
f.write("Total event executed")   
f.write(",") 
f.write("Pending Event")   
f.write(",")
f.write("Avg waiting time")   
f.write(",") 
#f.write("Avg Execution time")   
#f.write(",") 
f.write("Avg Transmission time")   
f.write(",") 
f.write("Total Efficiency")   
f.write(",") 
f.write("Total Cost")   
f.write(",") 
f.write("Delivered Efficiency")   
f.write(",") 
f.write("Total Offload")   
f.write(",")
f.write("Locally Executed")   
f.write(",")
f.write("Offload Execution")   
f.write(",")
f.write("Good decisions")
f.write(",")
f.write("Bad decisions")
f.write(",")
f.write("Direct delivery")
f.write(",")
f.write("One-hop delivery")
f.write(",")
f.write("Two-hop delivery")
f.write(",")
f.write("Three hop delivery")  
f.write(",")
f.write("Four hop delivery")  
f.write(",")  
f.write("Failure delivery") 
f.write("\n")


#f.write(str("Local"))
###f.write(",")
#f.write(str(Total_offload_rsu)) # Total Offloaded, We will include total generated 
#f.write(str(Total_gen))
#f.write(",")
#f.write(str(LocalExec))
#f.write(",")
#f.write(str(Total_pending))
#f.write(",")
#f.write(str(AvgWaitintTime/(Total_in_localQueue )))
#f.write(",")
#f.write(str(Total_compute / Total_gen))
#f.write(",")
#f.write(str(0))
#f.write(",")
#f.write(str(0))
#f.write(",")
#f.write(str(0))
#f.write(",")
#f.write(str(0))
#f.write(",")
#f.write(str(0)) 
#f.write(",") 
#f.write(str(0))
#f.write("\n")


#f.write(str("V2V"))
#f.write(",")
######f.write(str(Total_offload_rsu)) # Total Offloaded, We will include total generated 
#f.write(str(Total_gen))
#f.write(",")
#f.write(str(LocalExec))
#f.write(",")
#f.write(str(Total_pendingV2V))
#f.write(",")
#f.write(str(AvgWaitintTimeV2V/(Total_recvV2V)))
#f.write(",")
#f.write(str(Total_computeV2V / Total_gen))
#f.write(",")
#f.write(str(Ddelivery + oneHopDel + twoHopDel + threeHopDel))
#f.write(",")
#f.write(str(failureDel))
#f.write(",")
#f.write(str(oneHopDel))
#f.write(",")
#f.write(str(twoHopDel))
#f.write(",")
#f.write(str(threeHopDel)) 
#f.write(",") 
#f.write(str(failureDel))
#f.write("\n")

f.write(str("V2V"))
f.write(",")
f.write("100")   
f.write(",") 
#f.write(str(Total_offload_rsu)) # Total Offloaded, We will include total generated 
f.write(str(Total_gen))
f.write(",")
f.write(str(LocalExec + V2VExec))
f.write(",")
f.write(str(Total_pending))
f.write(",")
try:
   A1 = AvgWaitintTime/(countTskExecutionQueueVeh)  #+ Total_recv, initially used
   f.write(str(A1))
   f.write(",")
except ZeroDivisionError:
   A1 = AvgWaitintTime
   f.write(str(A1))
   f.write(",")
#try:
 #  T1 = execution_TimeV2V/Total_offload
 #  f.write(str(T1))
  # f.write(",")
#except ZeroDivisionError:
  # T1 = execution_TimeV2V
  # f.write(str(T1))
  # f.write(",")   
   
try:
   A11 = AvgTransmissionTime/Total_offload
   f.write(str(A11))
   f.write(",")
except ZeroDivisionError:
   A11 = AvgTransmissionTime
   f.write(str(A11))
   f.write(",")
   
try:
   #B1 = (V2VExec + Total_compute) / Total_gen
   B1 = (V2VExec + Total_compute)
   f.write(str(B1))
   f.write(",") 
except ZeroDivisionError:
   B1 = Total_compute
   f.write(str(B1))
   f.write(",")   

SC1 = (A1 + A11)
f.write(str(SC1))   
f.write(",")     
try:
   #C1 = Ddelivery + oneHopDel + twoHopDel + threeHopDel
   C1 = Ddelivery + oneHopDel + twoHopDel + threeHopDel + fourHopDel
   #C1 = (Ddelivery + oneHopDel + twoHopDel + threeHopDel)/Total_offload
   f.write(str(C1))  
   f.write(",") 
except ZeroDivisionError:  
   #C1 = Ddelivery + oneHopDel + twoHopDel + threeHopDel
   C1 = Ddelivery + oneHopDel + twoHopDel + threeHopDel + fourHopDel 
   f.write(str(C1))   
   f.write(",")    
   
f.write(str(Total_offload))  
f.write(",")
f.write(str(LocalExec)) # Anothe option Local execution = LocalExec - Total_offload 
f.write(",")
f.write(str(V2VExec))   
f.write(",")
#f.write(str(Ddelivery + oneHopDel + twoHopDel + threeHopDel))
f.write(str(Ddelivery + oneHopDel + twoHopDel + threeHopDel + fourHopDel))
f.write(",")
f.write(str(failureDel))
f.write(",")
f.write(str(Ddelivery))
f.write(",")
f.write(str(oneHopDel))
f.write(",")
f.write(str(twoHopDel))
f.write(",")
f.write(str(threeHopDel)) 
f.write(",") 
f.write(str(fourHopDel))
f.write(",")
f.write(str(failureDel))
f.write("\n")

    
f.write(str("V2 RSU ONLY "))
f.write(",")
f.write("100")   
f.write(",")
#f.write(str(Total_offload)) # Total Offloaded, We will include total generated 
f.write(str(Total_gen))
f.write(",")
#f.write(str(total_local_V2RSU))
f.write(str(LocalExec + V2VExec + total_local_V2RSU))
f.write(",")
f.write(str(Total_pending_rsu))
f.write(",")
try:
   A2 = AvgWaitintTime_rsu/Total_recv_rsu
   f.write(str(A2))
   f.write(",") 
except ZeroDivisionError:
   A2 = AvgWaitintTime_rsu
   f.write(str(A2))
   f.write(",")
#try:
#   T2 = execution_TimeRSU/Total_offload_rsu
 #  f.write(str(T2))
 #  f.write(",") 
#except ZeroDivisionError:
 #  T2 = execution_TimeRSU
 #  f.write(str(T2))
 #  f.write(",")      
try:
   A22 = AvgTransmissionTime_rsu/Total_offload_rsu
   f.write(str(A22))
   f.write(",") 
except ZeroDivisionError:
   A22 =  AvgTransmissionTime_rsu
   f.write(str(A22))
   f.write(",")

try:
   #B2 = Total_compute_rsu / Total_recv_rsu  
   #B2 = (LocalExec + Total_compute_rsu) / Total_gen 
   B2 = (Total_compute_rsu)
   f.write(str(B2))
   f.write(",")
except ZeroDivisionError:
   B2 = 1
   f.write(str(B2))
   f.write(",")
   
SC2 = (A2 + A22)
f.write(str(SC2))   
f.write(",")       
   
try:
   #C2 =(Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu)/total_local_V2RSU
  # C2 =(Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu)
   C2 =(Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu + fourHopDel_rsu)
   f.write(str(C2))  
   f.write(",") 
except ZeroDivisionError:
   #C2 = Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu
   C2 =(Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu + fourHopDel_rsu)
   f.write(str(C2))  
   f.write(",")    
f.write(str(Total_offload_rsu))  
f.write(",")
f.write("0")   
f.write(",")
f.write(str(total_local_V2RSU))  
f.write(",")
#f.write(str(Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu ))
f.write(str(Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu + fourHopDel_rsu))
f.write(",")
f.write(str(failureDel_rsu))
f.write(",")
f.write(str(Ddelivery_rsu))
f.write(",")
f.write(str(oneHopDel_rsu))
f.write(",")
f.write(str(twoHopDel_rsu ))
f.write(",")
f.write(str(threeHopDel_rsu)) 
f.write(",") 
f.write(str(fourHopDel_rsu))
f.write(",")
f.write(str(failureDel_rsu))
f.write("\n")

 

f.write(str("V2 Park ONLY "))
f.write(",")
f.write("100")   
f.write(",")
#f.write(str(Total_offload_Park)) # Total Offloaded, We will include total generated 
f.write(str(Total_gen))
f.write(",")
#f.write(str(total_local_V2Park)) # Old suitable, task offloaded only on park
f.write(str(LocalExec + V2VExec + total_local_V2Park))
f.write(",")
f.write(str(Total_pending_Park))
f.write(",")
try:
   A3 = AvgWaitintTime_Park/Total_recv_Park
   f.write(str(A3))
   f.write(",")
except ZeroDivisionError:
   A3 = AvgWaitintTime_Park
   f.write(str(A3))
   f.write(",")
   
#try:
 #  T3 = execution_TimePark/Total_offload_Park
  # f.write(str(T3))
   #f.write(",")
#except ZeroDivisionError:
 #  T3 = execution_TimePark
  # f.write(str(T3))
  # f.write(",")
        
try:
   A33 = AvgTransmissionTime_Park/Total_offload_Park
   f.write(str(A33))
   f.write(",")
except ZeroDivisionError:
   A33 = AvgTransmissionTime_Park
   f.write(str(A33))
   f.write(",")

try:  
   #B3 = Total_compute_Park /Total_recv_Park
   #B3 = (LocalExec + Total_compute_Park)/Total_gen 
   B3 = (Total_compute_Park)
   f.write(str(B3))
   f.write(",")
except ZeroDivisionError:
   B3 = 1
   #B3 = total_local_V2V + Total_compute_Park
   f.write(str(B3))
   f.write(",")
   
SC3 = (A3 + A33)
f.write(str(SC3))   
f.write(",")      
   
try:
   #C3 =(Ddelivery_Park + oneHopDel_Park + twoHopDel_Park + threeHopDel_Park)/Total_offload_Park
   C3 =(Ddelivery_Park + oneHopDel_Park + twoHopDel_Park + threeHopDel_Park + fourHopDel_Park)
   f.write(str(C3)) 
   f.write(",")
except ZeroDivisionError:
   C3 = Ddelivery_Park + oneHopDel_Park + twoHopDel_Park + threeHopDel_Park + fourHopDel_Park
   f.write(str(C3)) 
   f.write(",") 
f.write(str(Total_offload_Park)) 
#f.write("0") 
f.write(",")
f.write("0") 
f.write(",")
f.write(str(total_local_V2Park))  
f.write(",")
f.write(str(Ddelivery_Park + oneHopDel_Park + twoHopDel_Park + threeHopDel_Park + fourHopDel_Park))
f.write(",")
f.write(str(failureDel_Park))
f.write(",")
f.write(str(Ddelivery_Park))
f.write(",")
f.write(str(oneHopDel_Park))
f.write(",")
f.write(str(twoHopDel_Park ))
f.write(",")
f.write(str(threeHopDel_Park)) 
f.write(",") 
f.write(str(fourHopDel_Park))
f.write(",")
f.write(str(failureDel_Park))
f.write("\n")


f.write(str("RSU to RSU "))
f.write(",")
f.write("100")   
f.write(",")
#f.write(str(Total_offload_rsuTorsu + total_Count_RSUtoRSUHandoff)) # Total Offloaded, We will include total generated , One chnaged in federation second due to deadline meet issue hand off is performed
f.write(str(Total_gen)) # 
f.write(",")
#f.write(str(Total_execution_RSUtoRSU)) #only RSU to RSU
f.write(str(LocalExec + V2VExec + total_local_V2RSU + Total_execution_RSUtoRSU))  
f.write(",")
f.write(str(Total_pending_RSUtoRSU))
f.write(",")
try:
   A4= AvgWaitintTime_RSUtoRSU/Total_recv_RSUtoRSU
   f.write(str(A4))
   f.write(",")
except ZeroDivisionError:
   A4= AvgWaitintTime_RSUtoRSU
   f.write(str(A4))
   f.write(",")
   
#try:
 #  T4= execution_TimeRSUtoRSU/Total_recv_RSUtoRSU
 #  f.write(str(T4))
  # f.write(",")
#except ZeroDivisionError:
 #  T4= execution_TimeRSUtoRSU
  # f.write(str(T4))
  # f.write(",")   
 
try:
  # A44=  AvgTransmissionTime_RSUtoRSU/Total_recv_RSUtoRSU 
   A44=  AvgTransmissionTime_RSUtoRSU/Total_recv_RSUtoRSU
   f.write(str(A44))
   f.write(",")
except ZeroDivisionError:
   A44= AvgTransmissionTime_RSUtoRSU
   f.write(str(A44))
   f.write(",")
    
try:
   
   #B4= (Total_compute + Total_compute_rsu + Total_Compute_RSUtoRSU)/Total_gen
   B4= (Total_Compute_RSUtoRSU)
   #B4= Total_Compute_RSUtoRSU/Total_recv_RSUtoRSU
   f.write(str(B4))
   f.write(",")
except ZeroDivisionError:
   B4 = 1
   f.write(str(B4))
   f.write(",")  

SC4 = (A4 + A44)
f.write(str(SC4))   
f.write(",")  
   
try:
   #C4 = (Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU)/Total_recv_RSUtoRSU
   #C4 =(C2 + ((Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU)/Total_recv_RSUtoRSU))/2
   #C4 =(Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU)
   C4 =(Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU + fourHopDel_RSUtoRSU)
   f.write(str(C4)) 
   f.write(",") 
except ZeroDivisionError:
   #C4 = Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU
   C4 =(Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU + fourHopDel_RSUtoRSU)
   f.write(str(C4))  
   f.write(",")    
f.write(str(Total_offload_rsuTorsu + total_Count_RSUtoRSUHandoff))   
f.write(",")
f.write("0") 
f.write(",")
f.write(str(Total_execution_RSUtoRSU)) 
f.write(",")
#f.write(str(Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU))
#f.write(str((Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu)+ (Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU) ))
f.write(str((Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu + fourHopDel_rsu)+ (Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU + fourHopDel_RSUtoRSU) ))
f.write(",")
f.write(str(failureDel_RSUtoRSU))
f.write(",")
f.write(str(Ddelivery_RSUtoRSU))
f.write(",")
f.write(str(oneHopDel_RSUtoRSU))
f.write(",")
f.write(str(twoHopDel_RSUtoRSU ))
f.write(",")
f.write(str(threeHopDel_RSUtoRSU))
f.write(",")
f.write(str(fourHopDel_RSUtoRSU))
f.write(",") 
f.write(str(failureDel_RSUtoRSU))
f.write("\n")


#Remember localExec is total compute
# Chekc if these are total stats 

f.write(str("V2ALL - V2X"))
f.write(",")
f.write("100")   
f.write(",")
f.write(str(Total_gen))
f.write(",")
f.write(str(LocalExec + V2VExec + Total_compute_rsu  + Total_compute_Park + Total_Compute_RSUtoRSU))
f.write(",")
f.write(str(Total_pending + Total_pending_rsu + Total_pending_Park + Total_pending_RSUtoRSU))
f.write(",")
f.write(str((A1 + A2 + A3 + A4)/4 )) ##########Total Wait
f.write(",")
#f.write(str((T1 + T2 + T3 + T4)/4 )) ##########Total Execution
#f.write(",")
f.write(str((A11 + A22 + A33 + A44)/4 )) ###### Total transmission
f.write(",")
f.write(str((B1 + B2 + B3 + B4)/Total_gen)) ######## Total Efficiecny
f.write(",")
f.write(str((SC1 + SC2 + SC3 + SC4)/4))  #Total Cost
f.write(",") 
f.write(str((C1 + C2 + C3 + C4)/(Total_offload + Total_offload_rsu + Total_offload_rsuTorsu + Total_offload_Park))) ########## Total delivered efficiency
f.write(",") 
#f.write(str(Total_offload + Total_offload_rsu + (Total_offload_rsuTorsu + total_Count_RSUtoRSUHandoff) + Total_offload_Park ))
f.write(str(Total_offload + Total_offload_rsu + Total_offload_Park ))
f.write(",")
f.write(str(LocalExec))
f.write(",") 
f.write(str(V2VExec + Total_compute_rsu  + Total_compute_Park + Total_Compute_RSUtoRSU)) 
f.write(",")
f.write(str((Ddelivery + oneHopDel + twoHopDel + threeHopDel + fourHopDel ) + (Ddelivery_rsu + oneHopDel_rsu + twoHopDel_rsu + threeHopDel_rsu + fourHopDel_rsu) + (Ddelivery_Park + oneHopDel_Park + twoHopDel_Park + threeHopDel_Park + fourHopDel_Park) + (Ddelivery_RSUtoRSU + oneHopDel_RSUtoRSU + twoHopDel_RSUtoRSU + threeHopDel_RSUtoRSU + fourHopDel_RSUtoRSU)))
f.write(",")
f.write(str((failureDel)+ (failureDel_rsu) + (failureDel_Park) + failureDel_RSUtoRSU))
f.write(",")
f.write(str(Ddelivery + Ddelivery_rsu + Ddelivery_Park + Ddelivery_RSUtoRSU))
f.write(",")
f.write(str(oneHopDel_rsu + oneHopDel + oneHopDel_Park + oneHopDel_RSUtoRSU))
f.write(",")
f.write(str(twoHopDel_rsu + twoHopDel + twoHopDel_Park + twoHopDel_RSUtoRSU ))
f.write(",")
f.write(str(threeHopDel_rsu + threeHopDel + threeHopDel_Park + threeHopDel_RSUtoRSU))
f.write(",") 
f.write(str(fourHopDel + fourHopDel_rsu + fourHopDel_Park + fourHopDel_RSUtoRSU))
f.write(",")
f.write(str(failureDel_rsu + failureDel + failureDel_Park + failureDel_RSUtoRSU))
f.write("\n")

  
f.close()
print("A4", A4, B4, C4)


f = open("dataset.csv","w")
i = 0
f.write("time")  
f.write(",") 
f.write("mips")
f.write(",") 
f.write("s_waitTime")   
f.write(",") 
f.write("d_waitTime")   
f.write(",") 
f.write("InRangeVeh")
f.write(",")
f.write("dist")
f.write(",")
f.write("decision")   
f.write("\n")


print ("Size of dataset is ", len(DataHolder))
for Data in DataHolder:
  f.write(str(Data.time))
  f.write(",")
  f.write(str(Data.mips))
  f.write(",")
  f.write(str(Data.s_waitTime))
  f.write(",")
  f.write(str(Data.d_waitTime))
  f.write(",")
  f.write(str(Data.inRangeVEH))
  f.write(",")
  f.write(str(Data.dist))
  f.write(",")
  f.write(str(Data.decision))
  f.write("\n")
  i += 1

f.close()



f = open("datasetDRL.csv","w")
i = 0
f.write("time")  
f.write(",") 
f.write("mips")
f.write(",") 
f.write("t_size")
f.write(",") 
f.write("cpu")
f.write(",") 
#f.write("r_v2v")
#f.write(",") 
#f.write("r_v2fog")
#f.write(",") 
#f.write("r_v2park")
#f.write(",") 
#f.write("t_relay")
#f.write(",") 
f.write("s_waitTime")   
f.write(",") 
f.write("d_waitTime")   
f.write(",") 
f.write("N_waitTimeV")
f.write(",")
f.write("N_waitTimeR")
f.write(",")
f.write("N_waitTimeP")
f.write(",")
f.write("N_transTimeV")
f.write(",")
f.write("N_transTimeR")
f.write(",")
f.write("N_transTimeP")
f.write(",")
f.write("S_speed")
f.write(",")
f.write("D_speed")
f.write(",")
f.write("InRangeVeh")
f.write(",")
f.write("distance")
f.write(",")
f.write("handoff")
f.write(",")
f.write("decision")   
f.write("\n")

print ("Size of dataset is ", len(DataHolder))
for Data in DataHolder:
  f.write(str(Data.time))
  f.write(",")
  f.write(str(Data.mips))
  f.write(",")
  f.write(str(Data.dataSize))
  f.write(",") 
  f.write(str(Data.cpuCycle))
  f.write(",") 
 # f.write(str(Data.tRateV2V))
  #f.write(",")
 # f.write(str(Data.tRateV2Fog))
 # f.write(",")
 # f.write(str(Data.tRateV2Park))
 # f.write(",")    
  #f.write(str(Data.relayTask))
  #f.write(",")     
  f.write(str(Data.s_waitTime))
  f.write(",")
  f.write(str(Data.d_waitTime))
  f.write(",")
  f.write(str(Data.n_waitV2V))
  f.write(",")
  f.write(str(Data.n_waitV2R))
  f.write(",")
  f.write(str(Data.n_waitV2P))
  f.write(",")
  f.write(str(Data.n_transV2V))
  f.write(",")
  f.write(str(Data.n_transV2R))
  f.write(",")
  f.write(str(Data.n_transV2P))
  f.write(",")
  f.write(str(Data.s_speed))
  f.write(",")
  f.write(str(Data.d_speed))
  f.write(",")
  f.write(str(Data.inRangeVEH))
  f.write(",")
  f.write(str(Data.dist))
  f.write(",")  
  f.write(str(Data.handoff))
  f.write(",")     
  f.write(str(Data.decision))
  f.write("\n")
  i += 1

f.close()





f = open("dataset_updated.csv","w")
i = 0
f.write("arrival rate")  
f.write(",") 
f.write("technique/Decision")
f.write(",") 
f.write("total_task_Generated")   
f.write(",") 
f.write("local_executiontasks")   
f.write(",")
f.write("v2v_offloadingTasks")   
f.write(",") 
f.write("v2v_failureDel")   
f.write(",") 
f.write("v2RSU_failureDel")
f.write(",")
f.write("task_returned_rsu")
f.write(",")
f.write("task_returned_park")
f.write(",")
f.write("Avg_waitTimeVehicle")
f.write(",")
f.write("Avg_waitTimeRSU")   
f.write(",")
f.write("Total_Avg_waitTimeRSU")   
f.write("\n")


print ("Size of dataset is ", len(DataHolder))
for Data in DataHolder:
  f.write(str(Data.time))
  f.write(",")
  f.write(str(Data.decision))
  f.write(",")
  f.write(str(Data.total_task_Generated))
  f.write(",")
  f.write(str(Data.local_executiontasks))
  f.write(",")
  f.write(str(Data.v2v_offloadingTasks))
  f.write(",")
  f.write(str(Data.v2v_failureDel))
  f.write(",")
  f.write(str(Data.v2RSU_failureDel))
  f.write(",")
  f.write(str(Data.task_returned_rsu))
  f.write(",")
  f.write(str(Data.task_returned_park))
  f.write(",")
  f.write(str(Data.Avg_waitTimeV2V)) 
  f.write(",")
  f.write(str(Data.Avg_waitTimeRSU))
  f.write(",")
  f.write(str(Data.Total_Avg_waitTime))
  f.write("\n")
  i += 1
  
f.close()