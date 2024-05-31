import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import importlib, argparse, os, sys, time
from pynput import keyboard

from softMotion.sm_continuum_manipulator import SMContinuumManipulator, apply_actuation_torques
from softMotion.utils import load_constrained_urdf
from softMotion.sm_manipulator_definition import SMManipulatorDefinition
from robotConfig import manipulator_definition

from simHelpers import simHelper
from pathing import get_velocity_vector





# Work on stiffing the robot
# Add containing at base of the robot



sys.path.append('./URDFS')

GRAVITY = -9.8

# Start pybullet sim
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, GRAVITY)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera( cameraDistance=3, cameraYaw=0, cameraPitch=-85, cameraTargetPosition=[0,0,12])


# load 
planeId = p.loadURDF("plane.urdf")

# Cylinder obstacles 
StartOr = p.getQuaternionFromEuler([0, 0, 0])
# To add more or less just append to this array below
positions_c = [[-1, -1, 0.1], [0, -1, 0.1], [1, -1, 0.1], [-0.5, -2, 0.1], [0.5, -2, 0.1]]
scale = 5
positions_c = [[scale * i[0], scale * i[1], scale * i[2]] for i in positions_c]
#simHelper.load_cylinders(physicsClient, positions_c, [StartOr for _ in range(len(positions_c))])
#simHelper.load_cylinders_fromfile("URDFS/obstacles/circle_locations.txt", physicsClient)

#sys.exit(0)

#simHelper.load_container(physicsClient, [[1.05, 52, 0], [-1.05, 52, 0]], [p.getQuaternionFromEuler([0, 0, 0])] * 2)


# Use robotConfig.py
#finger = SMContinuumManipulator(manipulator_definition, filename="")

# Create yaml config file
links_num = 20
simHelper.generate_config_yaml(links_num, 2.5,  "URDFS/new.yaml")

# Load yaml config file
#yml = SMManipulatorDefinition.from_file("./URDFS/benchmark_run_config.yaml")
yml = SMManipulatorDefinition.from_file("./URDFS/new.yaml")

# Create robot object
finger = SMContinuumManipulator(yml)

finger.load_to_pybullet(
    baseStartPos= [0, 30 + links_num, 0.1],
    baseStartOrn=p.getQuaternionFromEuler([-np.pi / 2, np.pi/2, np.pi]),
    baseConstraint="free",
    physicsClient=physicsClient,
)

p.changeDynamics(finger.bodyUniqueId, -1, lateralFriction=2, restitution=1)

import math
limit_range = [-math.pi/12, math.pi/12]
for i in range(p.getNumJoints(finger.bodyUniqueId)):
    p.changeDynamics(finger.bodyUniqueId, i, jointLowerLimit=limit_range[0], jointUpperLimit=limit_range[1])
    p.setJointMotorControl2(finger.bodyUniqueId, i, controlMode=p.VELOCITY_CONTROL, maxVelocity=1)
    
# Setup stuff from somo
time_step, n_steps, sim_time = 0.001, 2000000, 0
p.setTimeStep(time_step)

lam, omega = 1.0, 1.0
torque_fns = [
    #lambda t: 20 * np.sin(omega * t),
     0,
    lambda t: 20 * np.sin(omega * t - 1 * np.pi),
]  # - 0pi goes right, - pi goes left

real_time = time.time()
normal_forces = normal_forces_lastLink = time_plot = np.zeros((n_steps,))


velocity = [0.0, 0.0, 0.0]  
velocity_mult = 20
torque = 0
torque_multiplier = 0.2
#torque_multiplier = 200
# Keyboard input, will be deleted soon
def on_press(key):
    #global force
    global torque
    global velocity

    try:
       # print(str(key))
        if str(key) == "Key.right":
            torque = torque_multiplier
            #force[0] += 1# = [0, 0, 1]
        if str(key) == "Key.left":
            torque = -torque_multiplier
            #force[0] -= 1 #= [0, 0, -1]
        if str(key) == "Key.down":
            velocity = [0, -velocity_mult, 0]
            #force[1] -= 1 #[0, -1, 0]
        if str(key) == "Key.up":
            velocity = [0, velocity_mult, 0]
            #force[1] +=  1#[0, 1, 0]
        
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
    #print(force)
        
def on_release(key):
    #print('{0} released'.format(
        #key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False
listener = keyboard.Listener(
on_press=on_press,
on_release=on_release)
listener.start()

DURATION = 100000
joints_num = p.getNumJoints(finger.bodyUniqueId)
for i in range(DURATION):
    # Update robot physics stuff
    #finger.apply_actuation_torques(actuator_nrs=[1, 1, 1], axis_nrs=[0, 0, 0], actuation_torques=[torque, torque, torque])
    #import pdb; pdb.set_trace()
    #print(p.getNumJoints(finger.bodyUniqueId))
    #sys.exit(0)
    
    #print(p.getJointInfo(finger.bodyUniqueId, 50))
    # p.setJointMotorControl2(
    # bodyIndex=finger.bodyUniqueId,
    # jointIndex=90,
    # controlMode=p.TORQUE_CONTROL,
    # force=torque,
    # physicsClientId=physicsClient,
    #     )
    #print(p.getJointState(finger.bodyUniqueId, 90)[0])
    #apply_actuation_torques(finger.bodyUniqueId, [i for i in range(joints_num)], [torque for _ in range(joints_num)], lambda x: 0, physicsClient)
    p.setJointMotorControlArray(finger.bodyUniqueId, [i for i in range(joints_num)], p.POSITION_CONTROL, targetPositions=[torque for _ in range(joints_num)])
    #import pdb; pdb.set_trace()
    #finger.apply_passive_spring_torques([i for i in range(20)], [0 for i in range(20)])
    
    

    p.stepSimulation()
    time.sleep(1./480.)
    
    # if torque_fns[0] < 100:
    #     torque_fns[0] += 0.01
    #print(torque, velocity)
    #print(torque_fns[0])
    
    # At each frame apply an external force on the object
    # linvel, angvel = p.getBaseVelocity(finger.bodyUniqueId)
    # linvel = [round(x, 3) for x in linvel]
    # boxPos, boxOrn = p.getBasePositionAndOrientation(finger.bodyUniqueId)
    # p.applyExternalForce(objectUniqueId=finger.bodyUniqueId, linkIndex=-1,
    #                      forceObj=force, posObj=boxPos, flags=p.WORLD_FRAME)
    
    #vel = get_velocity_vector(i/500)
    #print(vel)
    
    #p.resetBaseVelocity(finger.bodyUniqueId, vel)
    
    p.resetBaseVelocity(finger.bodyUniqueId, velocity)


p.disconnect()