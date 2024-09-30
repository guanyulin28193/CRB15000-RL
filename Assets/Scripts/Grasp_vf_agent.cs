using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Barracuda;
using System;
using System.Threading.Tasks;
using Grpc.Core;
using Grpc.Net.Client;

public class GraspVfAgent : Agent
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
    public NNModel onnxModel; 
    private IWorker worker;
    private IKService.IKServiceClient client;
    private Channel channel;
    private bool isBeingDisabled = false;
    private bool SuccessfullyGrasped = false;

    // Ratio setting
    private float DistRatio = 0.0f;
    private float DistAwayRatio = 0.0f;
    private float AngleRewardRatio = 0.0f;
    private float Normalizer = 2000.0f; 

    // Init
    private float prevBest = 0.0f;
    private float BeginDistance = 0.0f;
    private float AngleReward = 0.0f;
    private float SuccessReward = 0.0f;
    private float stepReward = 0.0f;
    private float DistanceReward = 0.0f;
    private float CollidePenalty = 0.0f;
    private float CumulativeReward = 0.0f;
    private int Success_step = 0;
    private int requestCount = 0;
    Vector3 GraspOffset;
    private bool groundHit = false;
    private bool failed_grasp = false;
    private List<ArticulationBody> links = new();
    private int responseCount = 0;
    private float [] previours_response = new float[6];
    private float [] JointPositions = new float[6];
    private bool No_previours_response = true;
    private IKRequest request;
    public void Start()
    {
        Init();
    }

    private void Init()
    {
        if (links.Count == 0)  // 防止重复添加相同的链接
        {
            links.Add(Link1);
            links.Add(Link2);
            links.Add(Link3);
            links.Add(Link4);
            links.Add(Link5);
            links.Add(Link6);
        }

        // Initialize gRPC client
        if (channel == null || client == null)
        {
            channel = new Channel("127.0.0.1:50051", ChannelCredentials.Insecure);
            Debug.Log("Insertion gRPC channel has been initialized.");
            client = new IKService.IKServiceClient(channel);
        }

        if (worker == null)
        {
            var model = ModelLoader.Load(onnxModel);
            worker = WorkerFactory.CreateWorker(WorkerFactory.Type.CSharp, model);
            Debug.Log("Insertion worker has been initialized.");
        }
    }
    protected override void OnDisable()
    {
        isBeingDisabled = true;
        base.OnDisable();
    }

    protected override void OnEnable()
    {
        isBeingDisabled = false;
        base.OnEnable();
    }
    public void ResetAllAB()
    {
        links.ForEach(ab => ResetArticulationBody(ab));
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
        if (requestCount != 0)
        {
            Debug.Log("prevBest: " + prevBest);
            //Debug.Log("AngleReward: " + AngleReward);
            //Debug.Log("DistanceReward: " + DistanceReward);
            Debug.Log("SuccessReward: " + SuccessReward);
            //Debug.Log("First success at step: " + Success_step);
            //Debug.Log("CollidePenalty: " + CollidePenalty);
            //Debug.Log("GroundHit: " + groundHit);
            Debug.Log("StepReward: " + stepReward);
            Debug.Log("CumulativeReward: " + CumulativeReward);
            Debug.Log("RequestCount: " + requestCount);
            Debug.Log("responseCount: " + responseCount);
            Debug.Log("Log From last Episode End");
            Debug.Log(""); // Add a new line
            Debug.Log("Resetting the environment... New Episode Begins");
        }

        
        // Reset Rewards
        Success_step = 0;
        AngleReward = 0.0f;
        DistanceReward = 0.0f;
        stepReward = 0.0f;
        CollidePenalty = 0.0f;
        SuccessReward = 0.0f;
        CumulativeReward = 0.0f;
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
        Debug.Log("BeginDistance: " + BeginDistance);
    }

    public void CollectObservationBodyPart(ArticulationBody bp, VectorSensor sensor)
    {
        // Get velocities in the context of our base's space
        // Note: You can get these velocities in world space as well but it may not train as well.
        if (isBeingDisabled) return;
        sensor.AddObservation(transform.InverseTransformPoint(bp.transform.position));
        sensor.AddObservation((float)(bp.jointPosition[0] / (2 * Math.PI)));
        sensor.AddObservation(transform.InverseTransformDirection(bp.velocity));
        sensor.AddObservation(transform.InverseTransformDirection(bp.angularVelocity));
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (isBeingDisabled) return;
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
    public float[] GetObservationsToList()
    {
        var observations = new List<float>();

        Vector3 targetLocalPosition = transform.InverseTransformPoint(target.transform.transform.position);
        observations.Add(targetLocalPosition.x);
        observations.Add(targetLocalPosition.y);
        observations.Add(targetLocalPosition.z);

        observations.Add(target.transform.localRotation.eulerAngles.y / 360.0f);

        Vector3 gripperMidPoint = (transform.InverseTransformPoint(GripperA.transform.position) + 
                               transform.InverseTransformPoint(GripperB.transform.position)) / 2 
                               + GripperA.transform.up * 0.005f;
        observations.Add(gripperMidPoint.x);
        observations.Add(gripperMidPoint.y);
        observations.Add(gripperMidPoint.z);

        observations.Add(Vector3.Angle(GripperA.transform.up, Vector3.up) / 360.0f);

        foreach (var bodyPart in links)
        {
            GetObservationBodyPartToList(bodyPart, observations);
        }

        Vector3 boxLocalPosition = transform.InverseTransformPoint(box.transform.transform.position);
        observations.Add(boxLocalPosition.x);
        observations.Add(boxLocalPosition.y);
        //last element was added into actionMaskTensor

        return observations.ToArray();
    }

    public void GetObservationBodyPartToList(ArticulationBody bp, List<float> observations)
    {
        
        Vector3 localPosition = transform.InverseTransformPoint(bp.transform.position);
        observations.Add(localPosition.x);
        observations.Add(localPosition.y);
        observations.Add(localPosition.z);

        observations.Add((float)(bp.jointPosition[0] / (2 * Math.PI)));

        
        Vector3 localVelocity = transform.InverseTransformDirection(bp.velocity);
        observations.Add(localVelocity.x);
        observations.Add(localVelocity.y);
        observations.Add(localVelocity.z);

        Vector3 localAngularVelocity = transform.InverseTransformDirection(bp.angularVelocity);
        observations.Add(localAngularVelocity.x);
        observations.Add(localAngularVelocity.y);
        observations.Add(localAngularVelocity.z);
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
        
        //Debug.Log("Request: " + request);

        // Call the gRPC service
        requestCount++; //Count the number of requests sent
        var response = client.CalculateAnglesAsync(request).GetAwaiter().GetResult();
        
        // Set target to joints
        for (int i = 0; i < response.Angles.Count; i++)
        {   
            //Debug.Log("Setting joint " + i + " to " + response.Angles[i]);
            links[i].SetDriveTarget(ArticulationDriveAxis.X, response.Angles[i]);
            JointPositions[i] = (float)(links[i].jointPosition[0] * Mathf.Rad2Deg);
            //previours_response[i] = response.Angles[i];
        }
        //No_previours_response = false;

        responseCount += response.Angles.Count > 0 ? 1 : 0;

        //Debug.Log("Requests Sent:" + requestCount + " Responses applied" + responseCount);       
        
        //Step Reward -0.5f in total, decrease by 0.01f each step
        AddReward(-0.01f); 
        stepReward = stepReward - 0.01f;

        // Compute grasp reward
        Vector3 midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position)) / 2) + GripperA.transform.up * 0.005f;
        var distanceToTarget = Vector3.Distance(transform.InverseTransformPoint(target.transform.position), midpoint);
        float Gripper_angle = Vector3.Angle(GripperA.transform.up, Vector3.up);
        float Gripper_rotation = (float)(Link6.jointPosition[0] * 180 / Math.PI);
        float Target_rotation = target.transform.localRotation.eulerAngles.y;
        GraspOffset = Link6.transform.InverseTransformPoint(target.transform.position); //Link6 offset

        // Calculate the rotation difference between the gripper and the target
        float angleDiff = GetAngleDiff(Gripper_rotation,Target_rotation);

        // Reward if the gripper is in the grasping position && Gripper_angle < 190.0f && 170.0f < Gripper_angle
        if (target.GetComponent<Collider>().bounds.Contains(midpoint) && angleDiff < 25.0f && GraspOffset.z < 0.145f && GraspOffset.x < 0.02f && GraspOffset.x > -0.02f && GraspOffset.y < 0.055f && GraspOffset.y > -0.055f)
        {
            float Success_reward = 0.5f;
            /*Debug.Log("Offset to Link 6 is: " + localOffset);
            Debug.Log("Peg rotation is : " + Target_rotation);
            Debug.Log("Mid point Position is : " + midpoint);
            Debug.Log("Grasp end Target position : " + target.transform.position);*/
            AddReward(Success_reward);
            var vfReward = ComputeVfReward();
            vfReward = (vfReward - 0.75f)/2.21f - 0.5f; // Normalize the value function reward to -0.5 to 0.5 with min 0.75f and max 2.96f.
            Debug.Log("Value Function Reward: " + (vfReward));
            AddReward(vfReward);
            SuccessReward = SuccessReward + Success_reward;
            CumulativeReward = GetCumulativeReward();
            SuccessfullyGrasped = true;
        }

        float diff = BeginDistance - distanceToTarget;

        // Penalty if the target falls to the ground
        if (target.transform.localPosition.y < 0.1f)
        {
            GroundHitPenalty(GameObject.Find("Peg"), GameObject.Find("Ground"));
        }

        // Reward if the arm moves closer to target
        if (distanceToTarget > prevBest)
        {
            // Penalty if the arm moves away from the closest position to target
            float Dist_reward = DistAwayRatio * (prevBest - distanceToTarget) / Normalizer;
            AddReward(Dist_reward);
            DistanceReward = DistanceReward + Dist_reward;
        }
        else
        {
            // Reward if the arm moves closer to target
            float Dist_reward2 = DistRatio * diff / Normalizer;
            AddReward(Dist_reward2);
            DistanceReward = DistanceReward + Dist_reward2;
            prevBest = distanceToTarget;
        }

        // Penalty if the gripper is not in the right rotation
        float deviation = 50.0f;
        float Angle_reward = CalculatePenalty(angleDiff, deviation) * AngleRewardRatio / Normalizer;
        AddReward(-Angle_reward);
        AngleReward = AngleReward - Angle_reward;
        CumulativeReward = GetCumulativeReward();
    }

    public void GroundHitPenalty(GameObject CollidedObject, GameObject CollidedWith)
    {   
        /*if (requestCount!=0)
        {   
            float groundhitpen =-1500.0f / Normalizer;
            SetReward(groundhitpen);
            CollidePenalty += groundhitpen;
            groundHit = true;
            CumulativeReward = GetCumulativeReward();
            EndEpisode();
        }*/
    }

    public void PegHitPenalty(GameObject CollidedObject, GameObject CollidedWith)
    {
        /*if (CollidedWith.name == "Peg")
        {
            float peghitpen = -3.0f / Normalizer;
            AddReward(peghitpen);
            CollidePenalty += peghitpen;
        }
        else
        {
            float peghitground = -10.0f / Normalizer;
            AddReward(peghitground);
            CollidePenalty += peghitground;
            groundHit = true;
        }*/
    }

    float CalculatePenalty(float rotation_angle, float deviation)
    {
        float penalty = (float)Math.Exp(Math.Pow(rotation_angle, 2) / (2 * Math.Pow(deviation, 2)));
        return penalty - 1.0f;
    }
    float GetAngleDiff(float gripperRotation, float targetRotation)
    {
        float AngleDiff = Mathf.Abs(gripperRotation - targetRotation) % 180.0f;
        return Mathf.Min(AngleDiff, 180.0f - AngleDiff);
    }

    float ComputeVfReward()
    {
        var obsData = GetObservationsToList();
        List<float> LastElement = new List<float>();
        LastElement.Add(transform.InverseTransformPoint(box.transform.transform.position).z);

        //Debug.Log("Observations length: " + obsData.Length);
        var obsTensor = new Tensor(1, 1, 1, 70, obsData);
        var actionMaskTensor = new Tensor(1, 1, 1, 1, LastElement.ToArray());
        var recurrentTensor = new Tensor(1, 1, 1, 1);  
        var inputTensor = new Dictionary<string, Tensor>();
        inputTensor.Add("obs_0", obsTensor);
        inputTensor.Add("action_masks", actionMaskTensor);
        inputTensor.Add("recurrent_in", recurrentTensor);

        worker.Execute(inputTensor);

        var outputTensor = worker.PeekOutput("value_estimate");
        float vfReward = outputTensor[0];

        obsTensor.Dispose();
        actionMaskTensor.Dispose();
        recurrentTensor.Dispose();
        outputTensor.Dispose();

        return vfReward;
    }
    void OnApplicationQuit()
    {
        // Shutdown the gRPC channel
        if (channel != null)
        {
            channel.ShutdownAsync().Wait();
            Debug.Log("gRPC channel has been shutdown.");
        }
        if (worker != null)
        {
            worker.Dispose();
        }
    }

    public bool HasSuccessfullyGrasped()
    {
        return SuccessfullyGrasped;
    }
    public bool HasFailedGrasp()
    {
        return failed_grasp;
    }
    public int GetRequestCount()
    {
        return requestCount;
    }
    public Vector3 GetGraspOffset()
    {
        return GraspOffset;
    }
    public float[] GetJointAngles()
    {
        return JointPositions;
    }
    public void Resetter()
    {
        AngleReward = 0.0f;
        DistanceReward = 0.0f;
        CollidePenalty = 0.0f;
        SuccessReward = 0.0f;
        CumulativeReward = 0.0f;
        SuccessfullyGrasped = false;
        failed_grasp = false;
        groundHit = false;
        requestCount = 0;
        responseCount = 0;
    }

}