using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;
using System.Threading.Tasks;
using Grpc.Core;
using Grpc.Net.Client;

public class PlatformAgent : Agent
{
    public Transform target; //Target the agent will try to grasp.
    public Transform box;

    [Header("Body Parts")] public ArticulationBody Link1;
    public ArticulationBody Link2;
    public ArticulationBody Link3;
    public ArticulationBody Link4;
    public ArticulationBody Link5;
    public ArticulationBody Link6;
    public ArticulationBody GripperA;
    public ArticulationBody GripperB;
    private IKService.IKServiceClient client;
    private Channel channel;

    // Ratio setting
    private float DistRatio = 500.0f;
    private float DistAwayRatio = 100.0f;
    private float AngleRatio = 7.5f;
    private float SpeedRatio = 0.0f;
    private float Dist_Speed_Ratio = 2.5f;
    private const float stepPenalty = 0.0f;
    private float EnergyPenaltyFactor = -0.0f; // Factor for energy penalty

    // Init
    private float prevBest = 0.0f;
    private float BeginDistance = 0.0f;
    private float SpeedReward = 0.0f;
    private float AngleReward = 0.0f;
    private float SuccessReward = 0.0f;
    private float DistanceReward = 0.0f;
    private float StepReward = 0.0f;
    private float EnergyPenalty = 0.0f;
    private float CollidePenalty = 0.0f;
    private float CumulativeReward = 0.0f;
    private int fingerHitFloor = 0;
    private int requestCount = 0;
    private bool groundHit = false;
    private List<ArticulationBody> links = new();
    private int responseCount = 0;
    private float [] previours_response = new float[6];
    private bool No_previours_response = true;
    private IKRequest request;
    public void Start()
    {
        links.Add(Link1);
        links.Add(Link2);
        links.Add(Link3);
        links.Add(Link4);
        links.Add(Link5);
        links.Add(Link6);
        //links.Add(GripperA);
        //links.Add(GripperB);

        // Initialize gRPC client
        channel = new Channel("127.0.0.1:50051", ChannelCredentials.Insecure);
        client = new IKService.IKServiceClient(channel);
    }

    private void ResetArticulationBody(ArticulationBody articulationBody)
    {
        articulationBody.jointPosition = new ArticulationReducedSpace(0f);
        articulationBody.jointForce = new ArticulationReducedSpace(0f);
        articulationBody.jointVelocity = new ArticulationReducedSpace(0f);
        articulationBody.velocity = Vector3.zero;
        articulationBody.angularVelocity = Vector3.zero;
        articulationBody.SetDriveTarget(ArticulationDriveAxis.X, 0.0f);
    }

    public override void OnEpisodeBegin()
    {
        // Log From last Episode
        Debug.Log("BeginDistance: " + BeginDistance);
        Debug.Log("prevBest: " + prevBest);
        Debug.Log("SpeedReward: " + SpeedReward);
        Debug.Log("AngleReward: " + AngleReward);
        Debug.Log("DistanceReward: " + DistanceReward);
        Debug.Log("SuccessReward: " + SuccessReward);
        Debug.Log("CollidePenalty: " + CollidePenalty);
        Debug.Log("StepReward: " + StepReward);
        Debug.Log("EnergyPenalty: " + EnergyPenalty);
        Debug.Log("GroundHit: " + groundHit);
        Debug.Log("CumulativeReward: " + CumulativeReward);
        Debug.Log("RequestCount: " + requestCount);
        Debug.Log("responseCount: " + responseCount);
        Debug.Log("Log From last Episode End");

        

        // Reset Rewards
        SpeedReward = 0.0f;
        AngleReward = 0.0f;
        DistanceReward = 0.0f;
        StepReward = 0.0f;
        CollidePenalty = 0.0f;
        SuccessReward = 0.0f;
        EnergyPenalty = 0.0f;
        CumulativeReward = 0.0f;
        fingerHitFloor = 0;
        groundHit = false;
        requestCount = 0;
        responseCount = 0;

        // Reset Articulation Bodies
        links.ForEach(ab => ResetArticulationBody(ab));
        ResetArticulationBody(GripperA);
        ResetArticulationBody(GripperB);
        

        // Random reset the peg position and rotation
        target.transform.localPosition = new Vector3(UnityEngine.Random.Range(-0.25f, 0.22f), 0.165f, UnityEngine.Random.Range(0.5f, 0.9f));
        target.transform.localRotation = Quaternion.Euler(0, UnityEngine.Random.Range(0, 360), 0);
        target.GetComponent<Rigidbody>().velocity = Vector3.zero;
        target.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        BeginDistance = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position)) / 2));
        prevBest = BeginDistance;
    }

    public void CollectObservationBodyPart(ArticulationBody bp, VectorSensor sensor)
    {
        // Get velocities in the context of our base's space
        // Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(transform.InverseTransformPoint(bp.transform.position));
        sensor.AddObservation((float)(bp.jointPosition[0] / (2 * Math.PI)));
        sensor.AddObservation(transform.InverseTransformDirection(bp.velocity));
        sensor.AddObservation(transform.InverseTransformDirection(bp.angularVelocity));
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.InverseTransformPoint(target.transform.transform.position));
        sensor.AddObservation(target.transform.localRotation.eulerAngles.y / 360.0f);
        sensor.AddObservation((((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position)) / 2) + GripperA.transform.up * 0.005f));
        // Add gripper angle as observation
        sensor.AddObservation(Vector3.Angle(GripperA.transform.up, Vector3.up) / 360.0f);
        foreach (var bodyPart in links)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
        sensor.AddObservation(transform.InverseTransformPoint(box.transform.transform.position));
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var continuousActions = actionBuffers.ContinuousActions;

        // Convert the target position to a format suitable for gRPC request
        if (No_previours_response)
        {
            var action_request = new float[] { continuousActions[0] , continuousActions[1], continuousActions[2], continuousActions[3],continuousActions[4], continuousActions[5]};
            request = new IKRequest { Position = { action_request } };
        }
        else
        {
            var action_request = new float[] { continuousActions[0] , continuousActions[1], continuousActions[2], continuousActions[3],continuousActions[4], continuousActions[5], previours_response[0], previours_response[1], previours_response[2], previours_response[3], previours_response[4], previours_response[5]};
            request = new IKRequest { Position = { action_request } };
        }
        
        Debug.Log("Request: " + request);

        // Call the gRPC service
        requestCount++; //Count the number of requests sent
        var response = client.CalculateAnglesAsync(request).GetAwaiter().GetResult();
        


        
        // Set target to joints
        for (int i = 0; i < response.Angles.Count; i++)
        {   
            //Debug.Log("Setting joint " + i + " to " + response.Angles[i]);
            links[i].SetDriveTarget(ArticulationDriveAxis.X, response.Angles[i]);
            //previours_response[i] = response.Angles[i];
        }
        //No_previours_response = false;

        responseCount += response.Angles.Count > 0 ? 1 : 0;

        //Debug.Log("Requests Sent:" + requestCount + " Responses applied" + responseCount);

        // Compute reward
        Vector3 midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position)) / 2) + GripperA.transform.up * 0.005f;
        var distanceToTarget = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), midpoint);
        float Gripper_angle = Vector3.Angle(GripperA.transform.up, Vector3.up);
        float Gripper_rotation = (float)(Link6.jointPosition[0] * 180 / Math.PI);
        float Target_rotation = target.transform.localRotation.eulerAngles.y;

        // Smooth speed limitation near the target
        float speedOfLink6 = Link6.velocity.magnitude;
        float speedLimitFactor = (float)Math.Tanh(distanceToTarget * 0.5f); // Use a tanh function for smooth limitation
        float desiredSpeed = speedLimitFactor * Dist_Speed_Ratio;
        if (speedOfLink6 > desiredSpeed && distanceToTarget < 0.5f)
        {
            float Speed_reward = -SpeedRatio * (speedOfLink6 - desiredSpeed);
            AddReward(Speed_reward / 1000.0f);
            SpeedReward = Speed_reward + SpeedReward;
        }

        // Calculate the rotation difference between the gripper and the target

        float angleDiff = GetAngleDiff(Gripper_rotation,Target_rotation);

        // Reward if the gripper is in the grasping position && Gripper_angle < 190.0f && 170.0f < Gripper_angle
        if (target.GetComponent<Collider>().bounds.Contains(midpoint) && angleDiff < 30.0f) 
        {
            float Success_reward = 20.0f;
            AddReward(20.0f / 1000.0f);
            SuccessReward = SuccessReward + Success_reward;
            //Debug.Log("Win!!!");
            //EndEpisode();
        }

        float diff = BeginDistance - distanceToTarget;

        // Penalty if the target falls to the ground
        if (target.transform.localPosition.y < 0.1f)
        {
            GroundHitPenalty();
        }

        // Reward if the arm moves closer to target
        if (distanceToTarget > prevBest)
        {
            // Penalty if the arm moves away from the closest position to target
            float Dist_reward = DistAwayRatio * (prevBest - distanceToTarget);
            AddReward(Dist_reward / 1000.0f);
            DistanceReward = DistanceReward + Dist_reward;
        }
        else
        {
            // Reward if the arm moves closer to target
            float Dist_reward2 = DistRatio * diff;
            AddReward(Dist_reward2 / 1000.0f);
            DistanceReward = DistanceReward + Dist_reward2;
            prevBest = distanceToTarget;
        }

        // Penalty if the gripper is not in the right rotation
        float deviation = 150.0f;
        float Angle_reward = AngleRatio * CalculatePenalty(Gripper_angle, angleDiff, deviation);
        AddReward(-Angle_reward / 1000.0f);
        AngleReward = AngleReward - Angle_reward;
        AddReward(stepPenalty / 1000.0f);
        StepReward = StepReward + stepPenalty;
    }

    public void GroundHitPenalty()
    {   
        if (requestCount!=0)
        {   
            float groundhitpen =-1000.0f/1000f;
            SetReward(groundhitpen);
            CollidePenalty += groundhitpen;
            groundHit = true;
            EndEpisode();
        }
    }

    public void PegHitPenalty(GameObject CollidedObject)
    {
        if (CollidedObject.name == "Peg")
        {
            float peghitpen = -3.0f / 1000f;
            AddReward(peghitpen);
            CollidePenalty += peghitpen;

            // EndEpisode();
        }
        else
        {
            float peghitground = -10.0f / 1000f;
            AddReward(peghitground);
            CollidePenalty += peghitground;
            fingerHitFloor += 1;
            groundHit = true;
            // EndEpisode();
        }
    }

    float CalculatePenalty(float Gripper_angle, float rotation_angle, float deviation)
    {
        float deviationFrom180 = Math.Abs(Gripper_angle - 180.0f);
        float penalty = (float)Math.Exp(Math.Pow(deviationFrom180, 2) / (2 * Math.Pow(deviation, 2)));
        float penalty2 = (float)Math.Exp(Math.Pow(rotation_angle, 2) / (2 * Math.Pow(deviation, 2)));

        return 0.7f * penalty + (1.3f*penalty2) - 2.0f;
    }

    void OnApplicationQuit()
    {
        // Shutdown the gRPC channel
        if (channel != null)
        {
            channel.ShutdownAsync().Wait();
            Debug.Log("gRPC channel has been shutdown.");
        }
    }
    float GetAngleDiff(float gripperRotation, float targetRotation)
    {
        float AngleDiff = Mathf.Abs(gripperRotation - targetRotation) % 180.0f;
        return Mathf.Min(AngleDiff, 180.0f - AngleDiff);
    }
}