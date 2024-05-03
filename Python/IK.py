import ikpy.chain
import numpy as np

def calculate_angles(target_position, target_orientation = None, orientation_mode="all"):
    Chain = ikpy.chain.Chain.from_urdf_file("./Assets/URDF/crb15000_5_95_gripper.urdf",active_links_mask=[False,True,True,True,True,True,True,False])
    if target_orientation == None:
        angles_degrees = np.degrees(Chain.inverse_kinematics(target_position))
    else:
        angles_degrees = np.degrees(Chain.inverse_kinematics(target_position, target_orientation, orientation_mode))
    angles_degrees[1] -= 90
    angles_degrees = angles_degrees[1:-1]
    return angles_degrees

#Position here is x,y,z, but in unity it is x,z,y so we need to swap the y and z values
#Inverse kinematics for end effector as the middle point of two fingers

if __name__ == '__main__':
    print(calculate_angles([-0.1892312,0.6445998,0.165], [0,0,1]))