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
    private float DistRatio = 200.0f;
    private float DistAwayRatio = 100.0f;
    private float Normalizer = 2000.0f;

    // Init
    private float prevBest = 0.0f;
    private float BeginDistance = 0.0f;
    private float AngleReward = 0.0f;
    private float SuccessReward = 0.0f;
    private float DistanceReward = 0.0f;
    private Vector3 midpoint;
    private float CollidePenalty = 0.0f;
    private float CumulativeReward = 0.0f;
    private int requestCount = 0;
    private bool groundHit = false;
    private List<ArticulationBody> links = new();
    private int responseCount = 0;
    private int First_CP_Step = 0;
    private float [] previours_response = new float[6];
    private bool No_previours_response = true;
    private bool Enable_BoxHitPenalty = true;
    private IKRequest request;
    private Vector3 HolePos = new Vector3(0.33f, 0.225f, 0.75f);
    bool[] checkpointVisited = new bool[13];
    int[] checkpointVisitedTimes = new int[13];
    public void Start()
    {
        links.Add(Link1);
        links.Add(Link2);
        links.Add(Link3);
        links.Add(Link4);
        links.Add(Link5);
        links.Add(Link6);

        // Initialize gRPC client
        channel = new Channel("127.0.0.1:50051", ChannelCredentials.Insecure);
        client = new IKService.IKServiceClient(channel);
    }

    private void ResetArticulationBody(ArticulationBody articulationBody)
    {
        articulationBody.SetDriveTarget(ArticulationDriveAxis.X, 0.0f);
        articulationBody.velocity = Vector3.zero;
        articulationBody.angularVelocity = Vector3.zero;
        articulationBody.jointForce = new ArticulationReducedSpace(0f);
        articulationBody.jointVelocity = new ArticulationReducedSpace(0f);
        articulationBody.jointPosition = new ArticulationReducedSpace(0f);
    }

    public override void OnEpisodeBegin()
    {
        int TotalCPVisited = 0; // Counting how many CP have been visited
        // Log From last Episode
        Debug.Log("EndDistance: " + prevBest);
        Debug.Log("AngleReward: " + AngleReward);
        Debug.Log("DistanceReward: " + DistanceReward);
        Debug.Log("SuccessReward: " + SuccessReward);
        Debug.Log("CollidePenalty: " + CollidePenalty);
        Debug.Log("GroundHit: " + groundHit);
        Debug.Log("CP Visited times: " + string.Join(", ", checkpointVisitedTimes));
        for (int i = 0; i < checkpointVisited.Length; i++)
        {
            if (checkpointVisited[i])
            {
                TotalCPVisited++;
                checkpointVisited[i] = false; //reset the checkpoint visited status
                checkpointVisitedTimes[i] = 0; //Reset the checkpoint visited times
            }

        }
        Debug.Log("CP Visited: " + TotalCPVisited);
        Debug.Log("CumulativeReward: " + CumulativeReward);
        Debug.Log("RequestCount: " + requestCount);
        Debug.Log("responseCount: " + responseCount);
        Debug.Log("First_CP_Step: " + First_CP_Step);
        Debug.Log("Log From last Episode End");
        Debug.Log(""); // Add a new line
        Debug.Log("Resetting the environment... New Episode Begins");
        

        // Reset Rewards
        AngleReward = 0.0f;
        DistanceReward = 0.0f;
        CollidePenalty = 0.0f;
        SuccessReward = 0.0f;
        CumulativeReward = 0.0f;
        First_CP_Step = 0;
        groundHit = false;
        requestCount = 0;
        responseCount = 0;
    
        // Remove the fixed joint if exists
        FixedJoint existingJoint = target.GetComponent<FixedJoint>();
        if (existingJoint != null)
        {
            Destroy(existingJoint);
        }

        // Random reset the target position between the gripper and connect with a fixed joint
        Vector3 Offset = new Vector3(0, UnityEngine.Random.Range(-0.05f, 0.05f), 0.145f);
        Debug.Log("Offset: " + Offset);
        Vector3 PegMidPointPosition = Link6.transform.TransformPoint(Offset);
        Vector3 PegGraspPotison = Link6.transform.TransformPoint(0, 0, 0.145f);
        Quaternion midpointRotation = Quaternion.LookRotation(PegGraspPotison-PegMidPointPosition, Link6.transform.up); //Calculate the rotation of the target
        target.transform.localRotation = midpointRotation;
        target.transform.localPosition = PegMidPointPosition;
        FixedJoint fixedJoint = target.AddComponent<FixedJoint>();
        fixedJoint.connectedArticulationBody = Link6;
        fixedJoint.enablePreprocessing = true;

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
        BeginDistance = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), HolePos);
        Debug.Log("BeginDistance: " + BeginDistance);
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
            //Debug.Log("Action Request Sent: " + string.Join(", ", action_request));
            request = new IKRequest { Position = { action_request } };
            // No_previours_response = false;
        }
        else
        {
            var action_request = new float[] { continuousActions[0] , continuousActions[1], continuousActions[2], continuousActions[3],continuousActions[4], continuousActions[5], previours_response[0], previours_response[1], previours_response[2], previours_response[3], previours_response[4], previours_response[5]};
            //Debug.Log("Action Request Sent: " + string.Join(", ", action_request));
            request = new IKRequest { Position = { action_request } };
        }
        // Call the gRPC service
        requestCount++; //Count the number of requests sent
        var response = client.CalculateAnglesAsync(request).GetAwaiter().GetResult();
        //Debug.Log("Got Response: " + string.Join(", ", response.Angles));
        
        // Set target to joints
        for (int i = 0; i < response.Angles.Count; i++)
        {   
            //Debug.Log("Setting joint " + i + " to " + response.Angles[i]);
            links[i].SetDriveTarget(ArticulationDriveAxis.X, response.Angles[i]);
            previours_response[i] = response.Angles[i];
        }
        

        responseCount += response.Angles.Count > 0 ? 1 : 0;

        //Debug.Log("Requests Sent:" + requestCount + " Responses applied" + responseCount);

        //////////////////////////////////////////////////////////Compute reward//////////////////////////////////////////////////////////////////////////////
        
        //The peg should be at 90 degrees to the hole. Penalty if the gripper is not in the right rotation
        float Target_rotation = target.transform.localRotation.eulerAngles.y;
        float Rot_diff_Target_rotation = Math.Abs(Target_rotation - 90.0f); 

        if (Rot_diff_Target_rotation > 90.0f)
        {
            Rot_diff_Target_rotation = Rot_diff_Target_rotation - 180.0f;
        }
        Rot_diff_Target_rotation = Math.Abs(Rot_diff_Target_rotation);

        float deviation = 50.0f;
        float Angle_reward = CalculatePenalty(Rot_diff_Target_rotation, deviation);
        float Angle_reward_Normalized = Angle_reward / Normalizer;
        AddReward(-Angle_reward_Normalized);
        AngleReward = AngleReward - Angle_reward_Normalized;

        // Reward if the target is in the hole, when deeper, the reward is higher because contains more virtual check points.
        for (int i = 0; i < 13; i++)
        {
            if (target.GetComponent<Collider>().bounds.Contains(new Vector3((0.33f + 0.01f*i), 0.225f, 0.75f)) & Rot_diff_Target_rotation < 15.0f)
            {
                if (checkpointVisited[i] == false) // Check if the checkpoint is visited
                {
                    if (i == 0)
                    {
                        First_CP_Step = responseCount; // Record at which step, the first checkpoint is visited
                    }
                    checkpointVisited[i] = true;
                }
                checkpointVisitedTimes[i]= checkpointVisitedTimes[i] + 1; // Count how many times the checkpoint is visited
                float Success_reward = 2.0f;
                float Success_reward_Normalized = Success_reward / Normalizer;
                AddReward(Success_reward_Normalized);
                SuccessReward = SuccessReward + Success_reward_Normalized;
            }
        }

        // Reward if the arm moves closer to target
        var distanceToTarget = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), HolePos);
        float diff = BeginDistance - distanceToTarget;
        if (distanceToTarget > prevBest)
        {
            // Penalty if the arm moves away from the closest position to target
            float Dist_reward = DistAwayRatio * (prevBest - distanceToTarget);
            float Dist_reward_Normalized = Dist_reward / (1.5f * Normalizer);
            AddReward(Dist_reward_Normalized);
            DistanceReward = DistanceReward + Dist_reward_Normalized;
        }
        else
        {
            // Reward if the arm moves closer to target
            float Dist_reward2 = DistRatio * diff;
            float Dist_reward2_Normalized = Dist_reward2 / (1.5f * Normalizer);
            AddReward(Dist_reward2_Normalized);
            DistanceReward = DistanceReward + Dist_reward2_Normalized;
            prevBest = distanceToTarget;
        }

        CumulativeReward = GetCumulativeReward();
    }

    public void GroundHitPenalty(GameObject CollidedObject, GameObject CollidedWith)
    {   
        if (requestCount!=0)
        {   
            float groundhitpen = -0.8f + requestCount * 0.016f; //Penalty decrese with the number of requests sent
            AddReward(groundhitpen);
            CumulativeReward = GetCumulativeReward();
            Debug.Log(CollidedObject.name + " collided with " + CollidedWith.name + " Penalty: " + groundhitpen);
            CollidePenalty += groundhitpen;
            groundHit = true;
            EndEpisode();
        }
    }

    public void PegHitPenalty(GameObject CollidedObject, GameObject CollidedWith)
    {
        
        if (CollidedWith.name == "BoxWithHole" && Enable_BoxHitPenalty)
        {
            float BoxHitPen = -0.5f / Normalizer;
            //Debug.Log(CollidedObject.name + " collided with " + CollidedWith.name + " Penalty: " + BoxHitPen);
            AddReward(BoxHitPen);
            CollidePenalty += BoxHitPen;
        }
        else if (CollidedWith.name == "Cube")
        {
            float peghitpen = -10.0f / Normalizer;
            Debug.Log(CollidedObject.name + " collided with " + CollidedWith.name + " Penalty: " + peghitpen);
            AddReward(peghitpen);
            CollidePenalty += peghitpen;
        }
    }

    float CalculatePenalty(float rotation_angle, float deviation)
    {
        float penalty = (float)Math.Exp(Math.Pow(rotation_angle, 2) / (2 * Math.Pow(deviation, 2)));
        return penalty - 1.0f;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actionsOutContinuousActions = actionsOut.ContinuousActions;
        actionsOutContinuousActions[0] = UnityEngine.Random.Range(-0.25f, 0.22f);
        actionsOutContinuousActions[1] = 0.2f;
        actionsOutContinuousActions[2] = UnityEngine.Random.Range(0.5f, 0.9f);
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

}