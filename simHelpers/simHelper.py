from softMotion.utils import load_constrained_urdf
import pybullet as p
# import yaml
import ruamel.yaml
import os

def load_cylinders(bullet_client, positions, orientations):
    assert len(positions) == len(orientations)
    
    for i in range(len(positions)):
        cyl_id, boxConstraintId2 = load_constrained_urdf(
            "./URDFS/cylinder.urdf",
            positions[i],
            orientations[i],
            physicsClient=bullet_client,
            static=1
        )
    p.changeDynamics(cyl_id, -1, lateralFriction=2, physicsClientId=bullet_client)

def load_container(bullet_client, positions, orientations):
    assert len(positions) == len(orientations)
    
    for i in range(len(positions)):
        cyl_id, boxConstraintId2 = load_constrained_urdf(
            "./URDFS/box.urdf",
            positions[i],
            orientations[i],
            physicsClient=bullet_client,
            static=1
        )
    p.changeDynamics(cyl_id, -1, lateralFriction=2, physicsClientId=bullet_client)


def generate_config_yaml(n_act, spacing, filename):
    yaml = ruamel.yaml.YAML()
    yaml.preserve_quotes = True
    #yaml.explicit_start = True
    
    
    n_segments = 5
    actuator_template = {
     "actuator_length": spacing,
     "n_segments": n_segments,
     "planar_flag": 1,
     "link_definition": {
         "shape_type": "stadium",
         "dimensions": [1., 1., spacing / n_segments],
         "mass": 0.00294,
         "inertial_values": [0.0152488, 0, 0, 0.0152488, 0, 0.0152488],
         "material_color": [0.10980392156862745, 0.3843137254901961, 0.8431372549019608, 1.0],
         "material_name": "navyblue"
     },
     "joint_definitions": [{
         "joint_type": "revolute",
         "axis": [1, 0, 0],
         "limits":  [-3.141592, 3.141592, 100, 3],
         "spring_stiffness": 87.4286,
         "joint_neutral_position": 0,
         "joint_control_limit_force": 0.4
     }]
    }
    

    base_def = {
        "shape_type": "box",
        "dimensions": [1, 1, 1.0],
        "mass": 0.00294,
        "inertial_values": [0.0152488, 0, 0, 0.0152488, 0, 0.0152488],
        "material_color": [0.10980392156862745, 0.3843137254901961, 0.8431372549019608, 1.0],
        "material_name": "navyblue"
    }
    
    tip_def = {
        "shape_type": "box",
        "dimensions": [1, 1, 0.25],
        "mass": 0.00294,
        "inertial_values": [0.0152488, 0, 0, 0.0152488, 0, 0.0152488],
        "material_color":  [0.0, 0.2, 0.4, 1.0],
        "material_name": "midnightblue"
    }
    
    
    URDF_name = "URDFS/cont_generated"
    data = {'manipulator_name': URDF_name, 
            "n_act": n_act,
            "actuator_definitions": [actuator_template for _ in range(n_act)],
            "base_definition": base_def,
            "tip_definition": tip_def
            }
    
    quoted_words = ["stadium", "navyblue", "revolute", "box", "midnightblue", URDF_name]
    
    tmp_file = filename[:-4] + "tmp.yaml"
    with open(tmp_file, "w+") as f:
        yaml.dump(data, f)
         #yaml.dump({"key": ["value1","value2","value3"]}, f, default_flow_style=True, sort_keys=False)
    

    with open(tmp_file, "r+") as f:
        with open(filename, "w+") as w:
            
            for line in f.readlines():
                # Assumes only one quoted word exists on a line at a time
                quote_flag = False
                for word in quoted_words:
                    if word in line:
                        quote_flag = True
                        position = line.find(word)
                        w.write(f"{line[:position]} \"{word}\" {line[position + len(word):]}")
                        
                if not quote_flag:
                    w.write(line)
                
    os.remove(tmp_file)

#generate_config_yaml(4, 5, "./test.yaml")