import grpc
import ik_pb2
import ik_pb2_grpc

def run():
    # Connect to the gRPC server
    channel = grpc.insecure_channel('localhost:50051')
    stub = ik_pb2_grpc.IKServiceStub(channel)

    # Create a request with a sample target position
    request = ik_pb2.IKRequest(position=[0.4, 0.5, 0.3 , -1 , 0 ,0])

    # Call the CalculateAngles method
    response = stub.CalculateAngles(request)
    print("Angles received from server: ", response.angles)

if __name__ == '__main__':
    run()