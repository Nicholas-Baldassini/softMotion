import numpy as np
import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import importlib
import argparse
import os
import sys

# path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
# sys.path.insert(0, path)
from softMotion.sm_continuum_manipulator import SMContinuumManipulator
from softMotion.utils import load_constrained_urdf


# xx major todo: fix proper packaging and importing, this is awful
# path = os.path.abspath(
#     os.path.join(os.path.dirname(__file__), "old_examples/exp_details")
# )
# sys.path.insert(0, path)

from robotConfig import manipulator_definition


DURATION = 10000
ALPHA = 300

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(
    0
)
planeId = p.loadURDF("plane.urdf")

StartOr = p.getQuaternionFromEuler([0, 0, 0])
positions_c = [[-1, -1, 0.1], [0, -1, 0.1], [1, -1, 0.1], [-0.5, -2, 0.1], [0.5, -2, 0.1]]
cylinder_ids = []
for i in range(len(positions_c)):
    cyl_id, boxConstraintId2 = load_constrained_urdf(
        "./URDFS/cylinder.urdf",
        positions_c[i],
        StartOr,
        physicsClient=physicsClient,
    )
    p.changeDynamics(cyl_id, -1, lateralFriction=2)
    cylinder_ids.append(cyl_id)

finger = SMContinuumManipulator(manipulator_definition)

finger.load_to_pybullet(
    baseStartPos= [0, 2, 0.1],
    baseStartOrn=p.getQuaternionFromEuler([-np.pi / 2, 0, np.pi]),
    baseConstraint="free",
    physicsClient=physicsClient,
)

p.changeDynamics(finger.bodyUniqueId, -1, lateralFriction=2, restitution=1)

time_step, n_steps, sim_time = 0.001, 2000000, 0
p.setTimeStep(time_step)

lam, omega = 1.0, 1.0
torque_fns = [
    lambda t: 20 * np.sin(omega * t),
    lambda t: 20 * np.sin(omega * t - 1 * np.pi),
]  # - 0pi goes right, - pi goes left

real_time = time.time()

normal_forces = normal_forces_lastLink = time_plot = np.zeros((n_steps,))


from pynput import keyboard



force = [0, 0, 0]
force = [0.0, 0.0, 0.0]  

def on_press(key):
    global force
    #print(str(key))
    try:
        #print('alphanumeric key {0} pressed'.format(
            #key.char))
        
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
    
    finger.apply_actuation_torques(
        actuator_nrs=[0], axis_nrs=[0], actuation_torques=[torque_fns[0](sim_time)]
        )
    finger.apply_actuation_torques(actuator_nrs=[0], axis_nrs=[1], actuation_torques=[0])


    p.stepSimulation()
    time.sleep(1./240.)
    
    linvel, angvel = p.getBaseVelocity(finger.bodyUniqueId)
    #print([round(x, 3) for x in linvel], angvel)
    boxPos, boxOrn = p.getBasePositionAndOrientation(finger.bodyUniqueId)


          
    p.applyExternalForce(objectUniqueId=finger.bodyUniqueId, linkIndex=-1,
                         forceObj=force, posObj=boxPos, flags=p.WORLD_FRAME)


p.disconnect()