import ikpy.chain
import numpy as np

def calculate_angles(target_position, target_orientation = None, orientation_mode=None):
    Chain = ikpy.chain.Chain.from_urdf_file("./Assets/URDF/crb15000_5_95_gripper.urdf",active_links_mask=[False,True,True,True,True,True,True,False])
    if target_orientation == None:
        angles_degrees = np.degrees(Chain.inverse_kinematics(target_position))
    else:
        angles_degrees = np.degrees(Chain.inverse_kinematics(target_position, target_orientation, orientation_mode))
    angles_degrees[1] -= 90
    angles_degrees = angles_degrees[1:-1]
    return angles_degrees