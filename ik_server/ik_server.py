import grpc
from concurrent import futures
import time
import ik_pb2
import ik_pb2_grpc
import ikpy.chain
import numpy as np
import os

# 位置格式转换函数
def convert_position(unity_position):
    return [unity_position[0], unity_position[2], unity_position[1]]

class IKService(ik_pb2_grpc.IKServiceServicer):
    def __init__(self, chain):
        self.chain = chain

    def CalculateAngles(self, request, context):
        target_position = list(request.position)
        
        #print(f"Received request with position: {target_position}")  # Add this line

        target_rotation = [target_position[3], target_position[4], target_position[5]]
        target_position = [target_position[0], target_position[2], target_position[1]] # Swap Y and Z because of Unity
        angles_degrees = np.degrees(self.chain.inverse_kinematics(target_position=target_position,target_orientation=target_rotation, orientation_mode="Z"))
        angles_degrees[1] -= 90
        angles_degrees = angles_degrees[1:-1]
        return ik_pb2.IKResponse(angles=angles_degrees)


def serve():
    # Load URDF once and initialize the chain
    urdf_path = os.path.join(os.path.dirname(__file__), '../Assets/URDF/crb15000_5_95_gripper.urdf')
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, active_links_mask=[False, True, True, True, True, True, True, False])

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=256))
    ik_pb2_grpc.add_IKServiceServicer_to_server(IKService(chain), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    print("Server started, listening on port 50051.")  # Confirm server started
    try:
        while True:
            time.sleep(86400)
    except KeyboardInterrupt:
        print("Server stopping...")  # Confirm server stopping
        server.stop(0)


if __name__ == '__main__':
    serve()
