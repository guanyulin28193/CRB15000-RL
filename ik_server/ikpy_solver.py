import ikpy.chain
import numpy as np
import os

def calculate_angles(target_position, target_orientation=None, orientation_mode="all"):
    urdf_path = os.path.join(os.path.dirname(__file__), '../Assets/URDF/crb15000_5_95_gripper.urdf')
    Chain = ikpy.chain.Chain.from_urdf_file(urdf_path, active_links_mask=[False, True, True, True, True, True, True, False])
    if target_orientation is None:
        angles_degrees = np.degrees(Chain.inverse_kinematics(target_position))
    else:
        angles_degrees = np.degrees(Chain.inverse_kinematics(target_position, target_orientation, orientation_mode))
    angles_degrees[1] -= 90
    angles_degrees = angles_degrees[1:-1]
    return angles_degrees

def convert_position(unity_position):
    return [unity_position[0], unity_position[2], unity_position[1]]
