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
    public bool resetingPos = false;
    public bool randomize = true;
    public bool initialized = false;
    private IKService.IKServiceClient client;
    private Vector3 InitPos;
    private List<ArticulationBody> links = new();
    private Vector3 midpoint;
    public bool rotating = false;

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
        if (resetingPos)
        {
            resetingPos = false;
            foreach (var link in links)
            {
                ResetArticulationBody(link);
            }
        }
        if (initialized == false)
        {
            initialized = true;
            FixedJoint existingJoint = target.GetComponent<FixedJoint>();
            if (existingJoint != null)
            {
                Destroy(existingJoint);
            }
            target.transform.localPosition = new Vector3(1.0f, 1.0f, 1.0f);
            //midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position))/2)+ upVector*0.007f;
            Vector3 DownEndPosition = new Vector3(0, UnityEngine.Random.Range(-0.05f, 0.05f), 0.145f);
            Vector3 MidPosition = new Vector3(0, 0, 0.145f);
            Vector3 worldDownEndPosition = Link6.transform.TransformPoint(DownEndPosition);
            Vector3 MidpointPosition = Link6.transform.TransformPoint(MidPosition);
            Quaternion midpointRotation = Quaternion.LookRotation(MidpointPosition-worldDownEndPosition, Link6.transform.up);
            target.transform.localRotation = midpointRotation;
            target.transform.localPosition = worldDownEndPosition;
            FixedJoint fixedJoint = target.AddComponent<FixedJoint>();
            fixedJoint.connectedArticulationBody = Link6;
            fixedJoint.enablePreprocessing = false;
        }
        if (randomize)
        {

            InitPos = new Vector3(UnityEngine.Random.Range(-0.25f, 0.22f), 0.2f, UnityEngine.Random.Range(0.5f, 0.9f));
            randomize = false;
            var Init_action = new float[] {InitPos.x, InitPos.y, InitPos.z, req4, req5, req6};
            Debug.Log("InitPos: " + string.Join(", ", Init_action));
            var Init_request = new IKRequest { Position = { Init_action } };
            var Init_response = client.CalculateAnglesAsync(Init_request).GetAwaiter().GetResult();
            for (int i = 0; i < Init_response.Angles.Count; i++)
            {
                links[i].jointPosition = new ArticulationReducedSpace(Init_response.Angles[i]* Mathf.Deg2Rad);
                links[i].SetDriveTarget(ArticulationDriveAxis.X, Init_response.Angles[i]);
            }
        }

        float Target_rotation = target.transform.localRotation.eulerAngles.y;
        float Rot_diff_Target_rotation = Math.Abs(Target_rotation - 90.0f); 

        if (Rot_diff_Target_rotation > 90.0f)
        {
            Rot_diff_Target_rotation = Rot_diff_Target_rotation - 180.0f;
        }
        Rot_diff_Target_rotation = Math.Abs(Rot_diff_Target_rotation);
        float Angle_reward = CalculatePenalty2(Rot_diff_Target_rotation, 50.0f);
        //Debug.Log("角度奖励/惩罚: " + Angle_reward);
        //Debug.Log("Link6: " + GripperA.transform.rotation.eulerAngles);
        //Debug.Log("PEG: " + target.transform.localRotation.eulerAngles);
        
        /*Vector3 DownEndPosition = new Vector3(0, 0f, 0.145f);
        Vector3 GripperOffset = new Vector3(req1, req2, req3);
        Vector3 GripperEND = GripperA.transform.TransformPoint(GripperOffset);
        Vector3 worldDownEndPosition = Link6.transform.TransformPoint(DownEndPosition);
        //target.transform.localPosition = GripperEND;
        Vector3 eulerRotation = Link6.transform.rotation.eulerAngles;
        Vector3 eulerRotation_Gripper = GripperA.transform.rotation.eulerAngles;
        //Debug.Log("Link6 rot: " + eulerRotation_Gripper);
        //target.transform.localRotation = Quaternion.Euler(eulerRotation_Gripper.y + 90.0f, eulerRotation_Gripper.z, 180.0f - eulerRotation_Gripper.x);
        //target.transform.localRotation = Quaternion.Euler(eulerRotation.x + 90.0f, eulerRotation.y, eulerRotation.z);*/
        
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

    float CalculatePenalty2(float rotation_angle, float deviation)
    {
        float penalty = Mathf.Exp(Mathf.Pow(rotation_angle, 2) / (2 * Mathf.Pow(deviation, 2)));
        return penalty-1;
    }
}

