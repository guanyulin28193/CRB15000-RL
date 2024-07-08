using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;
using System.Threading.Tasks;
using Grpc.Core;
using Grpc.Net.Client;

public class tester : MonoBehaviour
{
    [Header("Body Parts")] public ArticulationBody Link1;
    public ArticulationBody Link2;
    public ArticulationBody Link3;
    public ArticulationBody Link4;
    public ArticulationBody Link5;
    public ArticulationBody Link6;
    public ArticulationBody GripperA;
    public ArticulationBody GripperB;
    public float req1;
    public float req2;
    public float req3;
    public float req4;
    public float req5;
    public float req6;
    public GameObject target;
    public GameObject Midpoint;
    private IKService.IKServiceClient client;

    private List<ArticulationBody> links = new();

    // A flag to check if an async operation is already running
    private bool isProcessing;

    void Start()
    {
        var channel = new Channel("127.0.0.1:50052", ChannelCredentials.Insecure);
        client = new IKService.IKServiceClient(channel);
        Debug.Log("Client initialized.");

        links.Add(Link1);
        links.Add(Link2);
        links.Add(Link3);
        links.Add(Link4);
        links.Add(Link5);
        links.Add(Link6);
    }
    void FixedUpdate()
    {
        isProcessing = true;
        Vector3 upVector = GripperA.transform.up;
        Vector3 midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position))/2)+ upVector*0.005f; 
        Midpoint.transform.localPosition = midpoint;
        /*try
        {
            var localPos =target.transform.position;
            var localRot =target.transform.localRotation;
            var action_request = new float[] {req1, req2, req3, req4, req5, req6};
            Debug.Log("Request: " + string.Join(", ", action_request));

            var request = new IKRequest { Position = { action_request } };

            Debug.Log("Sending request...");
            var response = client.CalculateAnglesAsync(request).GetAwaiter().GetResult();
            Debug.Log("Response received.");
            Debug.Log("Response: " + response.Angles.Count);

            var angles = new float[response.Angles.Count];
            for (int i = 0; i < response.Angles.Count; i++)
            {
                angles[i] = response.Angles[i];
            }
            Debug.Log("Angles: " + string.Join(", ", angles));
            ApplyJointAngles(angles);
        }
        catch (RpcException ex)
        {
            Debug.LogError("gRPC error: " + ex.Status);
        }
        catch (Exception ex)
        {
            Debug.LogError("Exception in UpdateAsync: " + ex.Message);
        }
        finally
        {
            isProcessing = false;
        }*/
        float Gripper_rotation = (float)(Link6.jointPosition[0] * 180 / Math.PI);
        float Target_rotation = target.transform.localRotation.eulerAngles.y;
        float angleDiff = GetAngleDiff(Gripper_rotation, Target_rotation);
        float Gripper_angle = Vector3.Angle(GripperA.transform.up, Vector3.up);
        Debug.Log("angleDiff: " + angleDiff);
        Debug.Log("angleReward: " + -CalculatePenalty(Gripper_angle, angleDiff, 150.0f));
    }
    void ApplyJointAngles(float[] angles)
    {
        for (int i = 0; i < angles.Length; i++)
        {
       //     Debug.Log("Setting angle " + i + " to " + angles[i]);
            links[i].SetDriveTarget(ArticulationDriveAxis.X, angles[i]);
        }
    }
    float GetAngleDiff(float gripperRotation, float targetRotation)
    {
        float AngleDiff = Mathf.Abs(gripperRotation - targetRotation) % 180.0f;
        return Mathf.Min(AngleDiff, 180.0f - AngleDiff);
    }

    float CalculatePenalty(float Gripper_angle, float rotation_angle, float deviation)
    {
        float deviationFrom180 = Math.Abs(Gripper_angle - 180.0f);
        float penalty = (float)Math.Exp(Math.Pow(deviationFrom180, 2) / (2 * Math.Pow(deviation, 2)));
        float penalty2 = (float)Math.Exp(Math.Pow(rotation_angle, 2) / (2 * Math.Pow(deviation, 2)));

        return penalty + (penalty2) - 2.0f;
    }
}





        // 计算目标位置与中点位置之间的距离
        //var distanceToTarget = Vector3.Distance(target.transform.position, midpoint);

        // 访问每个ArticulationBody的ArticulationDrive，并读取当前位置
        /*float currentAngleA = GripperA.jointPosition[0];
        float currentAngleB = GripperB.jointPosition[0];
        float currentAngle2 = Link2.jointPosition[0];
        float currentAngle3 = Link3.jointPosition[0];
        float currentAngle4 = Link4.jointPosition[0];
        float currentAngle5 = Link5.jointPosition[0];
        float currentAngle6 = Link6.jointPosition[0];
        //Debug.Log((GripperA.transform.position + GripperB.transform.position)/2);
        Vector3 upVector = GripperA.transform.up;
        //Debug.Log("upVector: " + upVector);
        float rotationA = GripperA.transform.rotation.eulerAngles.y;

        float Gripper_angle = Vector3.Angle(upVector, Vector3.up);
        float deviation = 150.0f; // This is the standard deviation, adjust this value to your needs
        
        //float Gripper_rotation = GripperA.transform.rotation.eulerAngles.y;
        float Gripper_rotation = transform.InverseTransformPoint(Link6.transform.transform.localRotation.eulerAngles).y;
        currentAngle6 = (float)(currentAngle6 * 180 / Math.PI);
        //Debug.Log("Gripper_rotation: " + currentAngle6);
        float Target_rotation = target.transform.localRotation.eulerAngles.y;

        Vector3 midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position))/2)+ upVector*0.008f; 
        //Midpoint.transform.localPosition = midpoint;
        //Debug.Log("midpoint: " + midpoint);

        //Debug.Log("distance: " + Vector3.Distance(midpoint, target.transform.position));
        //Debug.Log("Speed:" + Link6.velocity.magnitude);
        //Debug.Log("reward" + (-(Link6.velocity.magnitude - Vector3.Distance(midpoint, target.transform.position)*2.0f)));

        float speedLimitFactor = (float)Math.Tanh(Vector3.Distance(midpoint, target.transform.position)*0.2f); // Use a tanh function for smooth limitation
        float desiredSpeed = speedLimitFactor * 2.0f;
        if (Link6.velocity.magnitude > desiredSpeed) 
        {
            float Speed_reward = - (Link6.velocity.magnitude - desiredSpeed);
            //Debug.Log("SpeedReward: " + Speed_reward);
        
        }
        
        //Debug.Log("desiredSpeed: " + desiredSpeed);

        float angleDiff = Mathf.Abs(currentAngle6 - Target_rotation);
        while (angleDiff > 150)
        {
            angleDiff -= 180;
        }

        //Debug.Log("angleDiff: " + angleDiff);
        //Debug.Log("angleReward: " + -CalculatePenalty(Gripper_angle, angleDiff, deviation));

        float CalculatePenalty(float Gripper_angle, float rotation_angle, float deviation)
        {
            // 计算夹持器角度与180度的偏差
            float deviationFrom180 = Math.Abs(Gripper_angle - 180.0f);

            // 计算惩罚值，偏差越大，惩罚值越大
            float penalty = (float)Math.Exp(Math.Pow(deviationFrom180, 2) / (2 * Math.Pow(deviation, 2)));
            float penalty2 = (float)Math.Exp(Math.Pow(rotation_angle, 2) / (2 * Math.Pow(deviation, 2)));

            // 返回总的惩罚值
            return penalty + penalty2-2.0f;
        }*/

