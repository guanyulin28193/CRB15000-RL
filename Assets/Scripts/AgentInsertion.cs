using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections;
using System;
using System.Threading.Tasks;
using Grpc.Core;
using Grpc.Net.Client;

public class AgentInsertion : Agent
{
    public GameObject target; //Target the agent will try to grasp.
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
    private float Normalizer = 1000.0f;

    // Init
    private float prevBest = 0.0f;
    private float BeginDistance = 0.0f;
    private float AngleReward = 0.0f;
    private float SuccessReward = 0.0f;
    private float DistanceReward = 0.0f;
    private Vector3 midpoint;
    private float CollidePenalty = 0.0f;
    private float CumulativeReward = 0.0f;
    private bool InitCompleted = true;
    private int requestCount = 0;
    private bool groundHit = false;
    private List<ArticulationBody> links = new();
    private int responseCount = 0;
    private float [] previours_response = new float[6];
    private bool No_previours_response = true;
    private IKRequest request;
    private Vector3 HolePos = new Vector3(0.33f, 0.225f, 0.75f);
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
        articulationBody.SetDriveTarget(ArticulationDriveAxis.X, 0.0f);
        articulationBody.velocity = Vector3.zero;
        articulationBody.angularVelocity = Vector3.zero;
        articulationBody.jointPosition = new ArticulationReducedSpace(0f);
        articulationBody.jointForce = new ArticulationReducedSpace(0f);
        articulationBody.jointVelocity = new ArticulationReducedSpace(0f);
        articulationBody.transform.localRotation = Quaternion.Euler(0, 0, 0);
    }

    public override void OnEpisodeBegin()
    {
        // Log From last Episode
        Debug.Log("BeginDistance: " + BeginDistance);
        Debug.Log("prevBest: " + prevBest);
        Debug.Log("AngleReward: " + AngleReward);
        Debug.Log("DistanceReward: " + DistanceReward);
        Debug.Log("SuccessReward: " + SuccessReward);
        Debug.Log("CollidePenalty: " + CollidePenalty);
        Debug.Log("GroundHit: " + groundHit);
        Debug.Log("CumulativeReward: " + CumulativeReward);
        Debug.Log("RequestCount: " + requestCount);
        Debug.Log("responseCount: " + responseCount);
        Debug.Log("Log From last Episode End");

        // Reset Rewards
        AngleReward = 0.0f;
        DistanceReward = 0.0f;
        CollidePenalty = 0.0f;
        SuccessReward = 0.0f;
        CumulativeReward = 0.0f;
        InitCompleted = false;
        groundHit = false;
        requestCount = 0;
        responseCount = 0;

        StartCoroutine(ResetSequentially());
        
        // Set the Peg position, between the gripper
        BeginDistance = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), HolePos);
        prevBest = BeginDistance;
    }
    private IEnumerator ResetSequentially()
    {
        foreach (var ab in links)
        {
            ResetArticulationBody(ab);
        }
        yield return new WaitForFixedUpdate();
        yield return new WaitForSeconds(0.5f);
        // Remove the fixed joint if exists
        FixedJoint existingJoint = target.GetComponent<FixedJoint>();
        if (existingJoint != null)
        {
            Destroy(existingJoint);
        }
        yield return new WaitForFixedUpdate();

        // Random reset the target position between the gripper and connect with a fixed joint
        Vector3 Offset = new Vector3(0, UnityEngine.Random.Range(-0.05f, 0.05f), 0.145f);
        Vector3 PegPosition = Link6.transform.TransformPoint(Offset);
        //Debug.Log("PegPosition: " + PegPosition);
        target.transform.localRotation = Quaternion.Euler(90.0f, 0, 0);
        target.transform.localPosition = PegPosition;
        target.GetComponent<Rigidbody>().velocity = Vector3.zero;
        target.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        FixedJoint fixedJoint = target.AddComponent<FixedJoint>();
        fixedJoint.connectedArticulationBody = Link6;
        fixedJoint.enablePreprocessing = true;
        yield return new WaitForFixedUpdate();

        Vector3 InitPos = new Vector3(UnityEngine.Random.Range(-0.25f, 0.22f), 0.2f, UnityEngine.Random.Range(0.5f, 0.9f));
        //Debug.Log("InitPos: " + InitPos);
        var Init_action = new float[] {InitPos.x, InitPos.y, InitPos.z, 0.0f, 0.0f, UnityEngine.Random.Range(-1f, 1f)};
        var Init_request = new IKRequest { Position = { Init_action } };
        var Init_response = client.CalculateAnglesAsync(Init_request).GetAwaiter().GetResult();
        for (int i = 0; i < Init_response.Angles.Count; i++)
        {
            links[i].jointPosition = new ArticulationReducedSpace(Init_response.Angles[i]* Mathf.Deg2Rad);
            links[i].SetDriveTarget(ArticulationDriveAxis.X, Init_response.Angles[i]);
        }
        yield return new WaitForFixedUpdate();
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
        //Debug.Log("OnActionReceived");
        midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position)) / 2) + GripperA.transform.up * 0.008f;
        if (InitCompleted == false)
        {
            /*if (target.GetComponent<Collider>().bounds.Contains(midpoint))
            {
                Debug.Log("Initlization complete");
                InitCompleted = true;
                //Random reset the gripper position
                Vector3 InitPos = new Vector3(UnityEngine.Random.Range(-0.25f, 0.22f), 0.2f, UnityEngine.Random.Range(0.5f, 0.9f));
                var Init_action = new float[] {InitPos.x, InitPos.y, InitPos.z, 0.0f, 0.0f, UnityEngine.Random.Range(-1f, 1f)};
                var Init_request = new IKRequest { Position = { Init_action } };
                var Init_response = client.CalculateAnglesAsync(Init_request).GetAwaiter().GetResult();
                for (int i = 0; i < Init_response.Angles.Count; i++)
                {
                    links[i].jointPosition = new ArticulationReducedSpace(Init_response.Angles[i]* Mathf.Deg2Rad);
                    links[i].SetDriveTarget(ArticulationDriveAxis.X, Init_response.Angles[i]);
                }

                // Set the Peg position, between the gripper
                BeginDistance = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), HolePos);
                prevBest = BeginDistance;
            }*/
        }
        else
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
            
            //Debug.Log("Request: " + request);

            // Call the gRPC service
            requestCount++; //Count the number of requests sent
            var response = client.CalculateAnglesAsync(request).GetAwaiter().GetResult();
            
            // Set target to joints
            for (int i = 0; i < response.Angles.Count; i++)
            {   
                //Debug.Log("Setting joint " + i + " to " + response.Angles[i]);
                links[i].SetDriveTarget(ArticulationDriveAxis.X, response.Angles[i]);
                previours_response[i] = response.Angles[i];
            }
            No_previours_response = false;

            responseCount += response.Angles.Count > 0 ? 1 : 0;

            //Debug.Log("Requests Sent:" + requestCount + " Responses applied" + responseCount);

            // Compute reward //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            var distanceToTarget = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), midpoint);
            float Gripper_angle = Vector3.Angle(GripperA.transform.up, Vector3.up);
            float Gripper_rotation = (float)(Link6.jointPosition[0] * 180 / Math.PI);
            float Target_rotation = target.transform.localRotation.eulerAngles.y;

            // Calculate the rotation difference between the gripper and the target

            float angleDiff = GetAngleDiff(Gripper_rotation,Target_rotation);

            // Reward if the gripper is in the grasping position && Gripper_angle < 190.0f && 170.0f < Gripper_angle
            if (target.GetComponent<Collider>().bounds.Contains(midpoint) && angleDiff < 30.0f) 
            {
                float Success_reward = 20.0f;
                float Success_reward_Normalized = Success_reward / Normalizer;
                AddReward(Success_reward_Normalized);
                SuccessReward = SuccessReward + Success_reward_Normalized;
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
                float Dist_reward_Normalized = Dist_reward / Normalizer;
                AddReward(Dist_reward_Normalized);
                DistanceReward = DistanceReward + Dist_reward_Normalized;
            }
            else
            {
                // Reward if the arm moves closer to target
                float Dist_reward2 = DistRatio * diff;
                float Dist_reward2_Normalized = Dist_reward2 / Normalizer;
                AddReward(Dist_reward2_Normalized);
                DistanceReward = DistanceReward + Dist_reward2_Normalized;
                prevBest = distanceToTarget;
            }

            // Penalty if the gripper is not in the right rotation
            float deviation = 150.0f;
            float Angle_reward = AngleRatio * CalculatePenalty(Gripper_angle, angleDiff, deviation);
            float Angle_reward_Normalized = Angle_reward / Normalizer;
            AddReward(-Angle_reward_Normalized);
            AngleReward = AngleReward - Angle_reward_Normalized;
        }
    }

    public void GroundHitPenalty()
    {   
        if (requestCount!=0)
        {   
            float groundhitpen =-1.0f;
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
            float peghitpen = -3.0f / Normalizer;
            AddReward(peghitpen);
            CollidePenalty += peghitpen;

            // EndEpisode();
        }
        else
        {
            float peghitground = -10.0f / Normalizer;
            AddReward(peghitground);
            CollidePenalty += peghitground;
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