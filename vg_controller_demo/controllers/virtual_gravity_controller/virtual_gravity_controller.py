""" VIRTUAL GRAVITY CONTROLLER FOR THE ROBOTIS OP-2 """

import numpy as np
import csv
import time, calendar
from controller import Supervisor

__author__="Timothy Jones, Sean Berta, Mkrtich Arslanyan, Palanivel Sathiya Moorthy"

# CREATE ROBOT INSTANCE
robot = Supervisor()

# GET TIMESTEP FOR THE CURRENT WORLD
TIME_STEP = int(robot.getBasicTimeStep())
DT=TIME_STEP/1000.0

M_COM=2.92532 # [kg] ROBOT MASS
M_1=0 # [kg] ROBOT MASS
G=9.82 # [m/s^2] GRAVITATIONAL ACCELERATION
L=[0.0335, 0.093, 0.093]
MAX_TORQUE=2.5
k=3.5 # TUNE THE VIRTUAL GRAVITY CONTROLLER

# ACCESS ACTUATORS AND ENCODERS
class Joint:
    def __init__(self, actuator, encoder):
        self._actuator=actuator
        self._encoder=encoder
        self._encoder.enable(TIME_STEP)
        
    def setTorque(self, u):
        self._actuator.setTorque(u)
        
    def setPosition(self, u):
        self._actuator.setPosition(u)
        
    def getPosition(self):
        return self._encoder.getValue()

JOINT_NAMES=("Ankle","LegLower")
legLeft=[Joint(robot.getDevice(f"{j}L"),robot.getDevice(f"{j}LS")) for j in JOINT_NAMES]
legRight=[Joint(robot.getDevice(f"{j}R"),robot.getDevice(f"{j}RS")) for j in JOINT_NAMES]

# ACCESS IMU
imu=robot.getDevice("InertialUnit")
imu.enable(TIME_STEP)

# SET INITIAL JOINT POSITIONS
legLeft[0].setPosition(0)
legRight[0].setPosition(0)
legLeft[1].setPosition(0)
legRight[1].setPosition(0)

# DISABLE LOW-LEVEL PIDs
legLeft[0].setTorque(0)
legRight[0].setTorque(0)
legLeft[1].setTorque(0)
legRight[1].setTorque(0)
    
def kin(q):
    x=L[1]*np.sin(q[0][0]) - L[2]*np.sin(q[1][0] - q[0][0])
    z=L[0] + L[1]*np.cos(q[0][0]) + L[2]*np.cos(q[1][0] - q[0][0])
    return np.array([[x],
                     [z]])
 
def g(q):
    return np.array([[(M_1 + M_COM)*G*L[1]*np.sin(q[0][0]) - M_COM*G*L[2]*np.sin(q[1][0] - q[0][0])],
                     [M_COM*G*L[2]*np.sin(q[1][0] - q[0][0])]])
    
def virtualGravity(q:float):
    return (1 + k)*g(q)

def constrain(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)

robotNode=robot.getFromDef("OP2")

# OPEN CSV FILE
current_GMT = time.gmtime()
ts = calendar.timegm(current_GMT)# GET CURRENT TIMESTAMP
filename=f'C:\\Users\\tim5v\\OneDrive - Worcester Polytechnic Institute (wpi.edu)\\Documents\\WPI\\Spring 2023\\Robot Control\\Final Project\\results\sim_results_{ts}.csv'
with open(filename,'w') as csvfile:
    writer=csv.writer(csvfile)
    fields=['t [s]','uRight_1 [N*cm]','uRight_2 [N*cm]','uLeft_1 [N*cm]','uLeft_2 [N*cm]','x_CoM [mm]','y_CoM [mm]','F_dist [N]']
    writer.writerow(fields)
        
    # RUN SIMULATION
    t=0.0
    F_dist=0 # EXTERNAL DISTRUBANCE
    F_MAX=5.0
    F_MIN=-1.8
    dir=1
    while robot.step(TIME_STEP) != -1:
        # APPLY INCREMENTALLY INCREASING FORCE TO CoM
        F_dist+=dir*0.01
        robotNode.addForceWithOffset([F_dist,0,0],[-0.0255815, -0.000317386, -0.0127962],True)
        if F_dist > F_MAX: 
            dir=-1
        elif F_dist < F_MIN:
            break
        
        # GET JOINT POSITIONS
        qLeft=np.array([[joint.getPosition()] for joint in legLeft])
        qRight=np.array([[joint.getPosition()] for joint in legRight])
        
        # CALCULATE CONTROL INPUT    
        uRight=virtualGravity(qRight)
        uLeft=virtualGravity(qLeft)
        # print(f"{uRight[0][0]*100} {uRight[1][0]*100} {uLeft[0][0]*100} {uLeft[1][0]*100}")
            
        # CONSTRAIN ANKLE TORQUES
        uRight[0][0]=constrain(uRight[0][0], -MAX_TORQUE, MAX_TORQUE) # CONSTRAIN TORQUE
        uLeft[0][0]=constrain(uLeft[0][0], -MAX_TORQUE, MAX_TORQUE) # CONSTRAIN TORQUE
        
        # CONSTRAIN KNEE TORQUES
        uRight[1][0]=constrain(uRight[1][0], -MAX_TORQUE, MAX_TORQUE) # CONSTRAIN TORQUE
        uLeft[1][0]=constrain(uLeft[1][0], -MAX_TORQUE, MAX_TORQUE) # CONSTRAIN TORQUE
        
        # SET RIGHT AND LEFT LEG JOINT TORQUES
        legLeft[0].setTorque(uRight[0][0])
        legRight[0].setTorque(uLeft[0][0])
        
        legLeft[1].setTorque(uRight[1][0])
        legRight[1].setTorque(uLeft[1][0])
        
        t+= DT # ADVANCE SIMULATION TIME
        
        # SAVE SIM DATA
        P_CoM=kin(qRight) # SAVE P_CoM
        simData=[t,uRight[0][0]*100,uRight[1][0]*100,uLeft[0][0]*100,uLeft[1][0]*100,P_CoM[0][0]*1000,P_CoM[1][0]*1000,F_dist]
        writer.writerow(simData)