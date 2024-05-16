import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import importlib, argparse, os, sys, time
from pynput import keyboard

from softMotion.sm_continuum_manipulator import SMContinuumManipulator
from softMotion.utils import load_constrained_urdf
from robotConfig import manipulator_definition

from simHelpers import simHelper


DURATION = 10000
GRAVITY = -9.8

# Start pybullet sim
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, GRAVITY)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(0)

# load 
planeId = p.loadURDF("plane.urdf")

# Cylinder obstacles 
StartOr = p.getQuaternionFromEuler([0, 0, 0])
# To add more or less just append to this array below
positions_c = [[-1, -1, 0.1], [0, -1, 0.1], [1, -1, 0.1], [-0.5, -2, 0.1], [0.5, -2, 0.1]]
simHelper.load_cylinders(physicsClient, positions_c, [StartOr for _ in range(len(positions_c))])

# Soft robot definition, to change its properties look at robotConfig.py
finger = SMContinuumManipulator(manipulator_definition)
finger.load_to_pybullet(
    baseStartPos= [0, 2, 0.1],
    baseStartOrn=p.getQuaternionFromEuler([-np.pi / 2, 0, np.pi]),
    baseConstraint="free",
    physicsClient=physicsClient,
)
p.changeDynamics(finger.bodyUniqueId, -1, lateralFriction=2, restitution=1)

# Setup stuff from somo
time_step, n_steps, sim_time = 0.001, 2000000, 0
p.setTimeStep(time_step)

lam, omega = 1.0, 1.0
torque_fns = [
    lambda t: 20 * np.sin(omega * t),
    lambda t: 20 * np.sin(omega * t - 1 * np.pi),
]  # - 0pi goes right, - pi goes left

real_time = time.time()
normal_forces = normal_forces_lastLink = time_plot = np.zeros((n_steps,))


force = [0.0, 0.0, 0.0]  

# Keyboard input, will be deleted soon
def on_press(key):
    global force

    try:
        print(str(key))
        if str(key) == "Key.up":
            force = [0, 0, 100]
        if str(key) == "Key.down":
            force = [0, 0, -100]
        if str(key) == "Key.left":
            force = [0, -100, 0]
        if str(key) == "Key.right":
            force = [0, 100, 0]
        
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
        
def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False
listener = keyboard.Listener(
on_press=on_press,
on_release=on_release)
listener.start()


for i in range(DURATION):
    # Update robot physics stuff
    finger.apply_actuation_torques(
        actuator_nrs=[0], axis_nrs=[0], actuation_torques=[torque_fns[0](sim_time)]
        )
    finger.apply_actuation_torques(actuator_nrs=[0], axis_nrs=[1], actuation_torques=[0])


    p.stepSimulation()
    time.sleep(1./240.)
    
    
    # At each frame apply an external force on the object
    linvel, angvel = p.getBaseVelocity(finger.bodyUniqueId)
    boxPos, boxOrn = p.getBasePositionAndOrientation(finger.bodyUniqueId)
    p.applyExternalForce(objectUniqueId=finger.bodyUniqueId, linkIndex=-1,
                         forceObj=force, posObj=boxPos, flags=p.WORLD_FRAME)


p.disconnect()