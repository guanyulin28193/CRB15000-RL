using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;
using System.Threading.Tasks;
using Grpc.Core;
using Grpc.Net.Client;
public class MoveToAgent : Agent
{
    public Transform target; //Target the agent will try to move to.
    public Transform Edge; //Edge of the ground

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
    private int EdgePosition;
    

    private float DistRatio = 200.0f;
    private float DistAwayRatio = 100.0f;
    private float Normalizer = 3000.0f; 


    private float prevBest = 0.0f;
    private float BeginDistance = 0.0f;
    private float DistanceReward = 0.0f;
    private float CollidePenalty = 0.0f;
    private float CumulativeReward = 0.0f;
    private int requestCount = 0;
    private bool groundHit = false;
    private List<ArticulationBody> links = new();
    private int responseCount = 0;
    private float [] previours_response = new float[6];
    private bool No_previours_response = true;
    private IKRequest request;
    void Start()
    {
        links.Add(Link1);
        links.Add(Link2);
        links.Add(Link3);
        links.Add(Link4);
        links.Add(Link5);
        links.Add(Link6);
        channel = new Channel("127.0.0.1:50051", ChannelCredentials.Insecure);
        Debug.Log("Client initialized.");
        client = new IKService.IKServiceClient(channel);
    }

    private void ResetArticulationBody(ArticulationBody articulationBody)
    {
        articulationBody.SetDriveTarget(ArticulationDriveAxis.X, 0.0f);
        articulationBody.jointPosition = new ArticulationReducedSpace(0f);
        articulationBody.jointForce = new ArticulationReducedSpace(0f);
        articulationBody.jointVelocity = new ArticulationReducedSpace(0f);
        articulationBody.velocity = Vector3.zero;
        articulationBody.angularVelocity = Vector3.zero;
    }
    public override void OnEpisodeBegin()
    {
        //Reset the agent to the starting position
        transform.localPosition = new Vector3(0, 0, 0);
        transform.localRotation = Quaternion.Euler(0, 0, 0);
        foreach (ArticulationBody link in links)
        {
            ResetArticulationBody(link);
        }
        ResetArticulationBody(GripperA);
        ResetArticulationBody(GripperB);
        groundHit = false;
        BeginDistance = Vector3.Distance(target.localPosition, transform.localPosition);
        CumulativeReward = 0.0f;
        DistanceReward = 0.0f;
        CollidePenalty = 0.0f;
        requestCount = 0;
        responseCount = 0;
        No_previours_response = true;

        EdgePosition = UnityEngine.Random.Range(0, 4); //Randomize the edge position
        if (EdgePosition == 0)
        {
            Edge.localPosition = new Vector3(0.34f, 0.1525f, 0.75f);
            Edge.localRotation = Quaternion.Euler(0, 0, 0);
        }
        else if (EdgePosition == 1)
        {
            Edge.localPosition = new Vector3(-0.34f, 0.1525f, 0.75f);
            Edge.localRotation = Quaternion.Euler(0, 0, 0);
        }
        else if (EdgePosition == 2)
        {
            Edge.localPosition = new Vector3(0.0f, 0.1525f, 0.41f);
            Edge.localRotation = Quaternion.Euler(0, 90, 0);
        }
        else if (EdgePosition == 3)
        {
            Edge.localPosition = new Vector3(0.0f, 0.1525f, 1.09f);
            Edge.localRotation = Quaternion.Euler(0, 90, 0);
        }
        target.transform.localPosition = new Vector3(UnityEngine.Random.Range(-0.15f, 0.15f), 0.185f, UnityEngine.Random.Range(0.6f, 0.9f));

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
        sensor.AddObservation((((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position)) / 2) + GripperA.transform.up * 0.005f)); // Add endeffector position as observation
        foreach (var bodyPart in links)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
        sensor.AddObservation(transform.InverseTransformPoint(Edge.transform.transform.position));
    }
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        /*var continuousActions = actionBuffers.ContinuousActions;

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
            links[i].SetDriveTarget(ArticulationDriveAxis.X, response.Angles[i]);
            //previours_response[i] = response.Angles[i];
        }
        //No_previours_response = false;

        responseCount += response.Angles.Count > 0 ? 1 : 0;

        // Compute reward*/
    }
}
