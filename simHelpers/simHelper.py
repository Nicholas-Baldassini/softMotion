from softMotion.utils import load_constrained_urdf
import pybullet as p

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
