using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;


public class LowLvlAgent : Agent
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
    private float prevBest;
    private float BeginDistance;
    private const float stepPenalty = -0.001f;


    
    private List<ArticulationBody> links = new();

    public override void Initialize()
    {
        links.Add(Link1);
        links.Add(Link2);
        links.Add(Link3);
        links.Add(Link4);
        links.Add(Link5);
        links.Add(Link6);
        //links.Add(GripperA);
        //links.Add(GripperB);
    }

    private void ResetArticulationBody(ArticulationBody articulationBody)
    {
        articulationBody.jointPosition = new ArticulationReducedSpace(0f);
        articulationBody.jointForce = new ArticulationReducedSpace(0f);
        articulationBody.jointVelocity = new ArticulationReducedSpace(0f);
    }

    public override void OnEpisodeBegin()
    {
        links.ForEach(ab => ResetArticulationBody(ab));
        //random reset the peg position and rotation
        target.transform.localPosition = new Vector3(UnityEngine.Random.Range(-0.25f, 0.22f), 0.165f, UnityEngine.Random.Range(0.5f, 0.9f));
        target.transform.localRotation = Quaternion.Euler(0, UnityEngine.Random.Range(0, 360), 0);
        target.GetComponent<Rigidbody>().velocity = Vector3.zero;
        target.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        BeginDistance = Vector3.Distance(transform.InverseTransformPoint(target.transform.transform.position), ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position))/2));
        prevBest = BeginDistance;
    }

    public void CollectObservationBodyPart(ArticulationBody bp, VectorSensor sensor)
    {
        //Get velocities in the context of our base's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(transform.InverseTransformPoint(bp.transform.position));
        sensor.AddObservation(bp.jointPosition[0]);
        sensor.AddObservation(transform.InverseTransformDirection(bp.velocity));
        sensor.AddObservation(transform.InverseTransformDirection(bp.angularVelocity));
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.InverseTransformPoint(target.transform.transform.position));
        sensor.AddObservation(target.transform.localRotation.eulerAngles.y);
        sensor.AddObservation(((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position))/2)+ GripperA.transform.up*0.008f);
        // Add gripper angle as observation
        sensor.AddObservation(Vector3.Angle(GripperA.transform.up, Vector3.up));
        foreach (var bodyPart in links)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var i = -1;
        var continuousActions = actionBuffers.ContinuousActions;
        Link1.SetDriveTarget(ArticulationDriveAxis.X, ComputeNormalizedDriveControl(Link1.xDrive,continuousActions[++i]));
        Link2.SetDriveTarget(ArticulationDriveAxis.X, ComputeNormalizedDriveControl(Link2.xDrive,continuousActions[++i]));
        Link3.SetDriveTarget(ArticulationDriveAxis.X, ComputeNormalizedDriveControl(Link3.xDrive,continuousActions[++i]));
        Link4.SetDriveTarget(ArticulationDriveAxis.X, ComputeNormalizedDriveControl(Link4.xDrive,continuousActions[++i]));
        Link5.SetDriveTarget(ArticulationDriveAxis.X, ComputeNormalizedDriveControl(Link5.xDrive,continuousActions[++i]));
        Link6.SetDriveTarget(ArticulationDriveAxis.X, ComputeNormalizedDriveControl(Link6.xDrive,continuousActions[++i]));
        

        // Compute reward
        Vector3 midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position))/2)+ GripperA.transform.up*0.008f; 

        var distanceToTarget = Vector3.Distance(transform.InverseTransformPoint(target.transform.transform.position), midpoint);
        float Gripper_angle = Vector3.Angle(GripperA.transform.up, Vector3.up);
        float Gripper_rotation = (float)(Link6.jointPosition[0] * 180 / Math.PI);
        float Target_rotation = target.transform.localRotation.eulerAngles.y;


        float angleDiff = Mathf.Abs(Gripper_rotation - Target_rotation);
        while (angleDiff > 150)
        {
            angleDiff -= 180;
        }
        

        if(target.GetComponent<Collider>().bounds.Contains(midpoint) &&  Gripper_angle < 190.0f && 170.0f < Gripper_angle && angleDiff < 5.0f && angleDiff > -5.0f)
        {
            AddReward(25.0f);
            //EndEpisode();
        }

        float diff = BeginDistance - distanceToTarget;
        if (target.transform.localPosition.y < 0.1f)
        {   
            GroundHitPenalty();
        }

        if (distanceToTarget > prevBest)
         {
            // Penalty if the arm moves away from the closest position to target
            AddReward(0.1f*(prevBest - distanceToTarget));
         }
         else
         {
            // Reward if the arm moves closer to target
            AddReward(0.1f*diff);
            prevBest = distanceToTarget;
         }
        

        //To make the gripper angle right
        float deviation = 15.0f;
        float reward = CalculateReward(Gripper_angle, angleDiff, deviation);
        AddReward(reward*0.05f);
        AddReward(stepPenalty);

    }
    public float ComputeNormalizedDriveControl(ArticulationDrive drive, float actionValue)
    {
        return drive.lowerLimit + (actionValue + 1) / 2 * (drive.upperLimit - drive.lowerLimit);
    }

    public void GroundHitPenalty()
   {
      AddReward(-50f);
      EndEpisode();
   }
   public void PegHitPenalty(GameObject CollidedObject)
   {
      if (CollidedObject.name == "Peg")
      {
         AddReward(-5.0f);
         //EndEpisode();
      }
      else
      {
         AddReward(-25.0f);
         //EndEpisode();
      }
      
   }


    float CalculateReward(float Gripper_angle, float rotation_algle, float deviation)
    {
        // 
        float deviationFrom180 = Math.Abs(Gripper_angle - 180.0f);

        // 
        float penalty = (float)Math.Exp(-Math.Pow(Math.Abs(Gripper_angle - 180.0f), 2) / (2 * Math.Pow(deviation, 2)));
        float penalty2 = (float)Math.Exp(-Math.Pow(rotation_algle, 2) / (2 * Math.Pow(deviation, 2)));

        // 
        return (penalty+penalty2);
    }



}