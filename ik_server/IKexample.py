import ikpy.chain
import numpy as np
import time
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

# 位置格式转换函数
def convert_position(unity_position):
    return [unity_position[0], unity_position[2], unity_position[1]]

# test ik speed
def test_ik_speed(target_position, iterations=1000):
    total_time = 0.0
    for _ in range(iterations):
        start_time = time.time()
        calculate_angles(target_position)
        end_time = time.time()
        total_time += end_time - start_time
    average_time = total_time / iterations
    print(f"IKPY平均求解时间: {average_time:.6f} 秒")

if __name__ == '__main__':
    unity_position = [-0.151, 0.1675, 0.539]
    target_position = convert_position(unity_position)
    print(f"角度: {calculate_angles(target_position)}")
    #test_ik_speed(target_position)
