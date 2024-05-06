using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class LowLvlAgent : Agent
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
    private float prevBest;
    private float BeginDistance;
    private const float stepPenalty = -0.0001f;
    private Vector3 EndEffectorPosition => (GripperA.transform.position + GripperB.transform.position) / 2;
    
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
        target.localPosition = new Vector3(Random.Range(-0.25f, 0.22f), 0.165f, Random.Range(0.35f, 0.75f));
        target.localRotation = Quaternion.Euler(0, Random.Range(0, 360), 0);
        BeginDistance = Vector3.Distance(target.position, (GripperA.transform.position + GripperB.transform.position) / 2);
        prevBest = BeginDistance;
    }

    public void CollectObservationBodyPart(ArticulationBody bp, VectorSensor sensor)
    {
        //Get velocities in the context of our base's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(transform.InverseTransformPoint(bp.transform.position));
        sensor.AddObservation(bp.jointPosition[0]);
        //sensor.AddObservation(transform.InverseTransformDirection(bp.velocity));
        //sensor.AddObservation(transform.InverseTransformDirection(bp.angularVelocity));
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.InverseTransformPoint(target.transform.position));
        sensor.AddObservation(transform.InverseTransformPoint(target.transform.localRotation.eulerAngles));
        sensor.AddObservation(transform.InverseTransformPoint(EndEffectorPosition));
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
        var distanceToTarget = Vector3.Distance(target.position, (GripperA.transform.position + GripperB.transform.position) / 2);

        float diff = BeginDistance - distanceToTarget;

        if (distanceToTarget < 0.05f)
        {
            AddReward(10.0f);
            EndEpisode();
        }

        if (distanceToTarget > prevBest)
         {
            // Penalty if the arm moves away from the closest position to target
            AddReward(0.15f*(prevBest - distanceToTarget));
         }
         else
         {
            // Reward if the arm moves closer to target
            AddReward(0.15f*diff);
            prevBest = distanceToTarget;
         }

         AddReward(stepPenalty);

    }
    public float ComputeNormalizedDriveControl(ArticulationDrive drive, float actionValue)
    {
        return drive.lowerLimit + (actionValue + 1) / 2 * (drive.upperLimit - drive.lowerLimit);
    }


}