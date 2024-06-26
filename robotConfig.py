import numpy as np

from softMotion.sm_manipulator_definition import SMManipulatorDefinition
from softMotion.sm_actuator_definition import SMActuatorDefinition
from softMotion.sm_link_definition import SMLinkDefinition
from softMotion.sm_joint_definition import SMJointDefinition
from softMotion.sm_continuum_manipulator import SMContinuumManipulator

# base_definition

# todo: minimize the content of this;
# todo have definitions saved in one file instead of in separate files

base_definition = None

joint_definition1 = {
    "joint_type": "revolute",
    "axis": [1, 0, 0],
    "limits": [-3.141592, 3.141592, 100, 3],
    "spring_stiffness": 100,
    "joint_neutral_position": 0,
    "neutral_axis_offset": [0.0, 0.05, 0.0, 0.0, 0.0, 0.0],
    "joint_control_limit_force": 1.0,
}

joint_definition2 = {
    "joint_type": "revolute",
    "axis": [0, 1, 0],
    "limits": [-3.141592, 3.141592, 100, 3],
    "spring_stiffness": 100,
    "joint_neutral_position": 0,
    "joint_control_limit_force": 1.0,
}

link_definition = {
    "shape_type": "stadium",
    "dimensions": [0.2, 0.2, 0.2],
    "mass": 35.0,
    "inertial_values": [1, 0, 0, 1, 0, 1],
    "material_color": [0.6, 0.0, 0.8, 1.0],
    "material_name": "green",
}

tip_definition = None


# Use NUMBER_OF_LINKS to define how long the robot should be
# some values dont work idk why

NUMBER_OF_LINKS = 10
actuator_definition = {
    "actuator_length": NUMBER_OF_LINKS / 5,
    "n_segments": NUMBER_OF_LINKS,
    "link_definition": link_definition,
    "joint_definitions": [joint_definition1, joint_definition2],
    "planar_flag": 0,
}


manipulator_definition = {
    "n_act": 1,
    "base_definition": base_definition,
    "actuator_definitions": [actuator_definition],
    "tip_definition": tip_definition,
    "manipulator_name": "finger",
    "urdf_filename": "./URDFS/finger.urdf",
}
