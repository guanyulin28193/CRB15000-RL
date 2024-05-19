using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class tester : MonoBehaviour
{
    public ArticulationBody GripperA;
    public ArticulationBody GripperB;
    public ArticulationBody Link2;
    public ArticulationBody Link3;
    public ArticulationBody Link4;
    public ArticulationBody Link5;
    public ArticulationBody Link6;
    public GameObject target;
    public GameObject Midpoint;
    

    void Update()
    {
        // 计算目标位置与中点位置之间的距离
        //var distanceToTarget = Vector3.Distance(target.transform.position, midpoint);

        // 访问每个ArticulationBody的ArticulationDrive，并读取当前位置
        float currentAngleA = GripperA.jointPosition[0];
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
        //Debug.Log("Gripper_angle: " + Gripper_angle);
        float deviation = 15.0f; // This is the standard deviation, adjust this value to your needs
        float angleReward = (float)Math.Exp(-Math.Pow(Gripper_angle - 180.0f, 2) / (2 * Math.Pow(deviation, 2)));
        
        //float Gripper_rotation = GripperA.transform.rotation.eulerAngles.y;
        float Gripper_rotation = transform.InverseTransformPoint(Link6.transform.transform.localRotation.eulerAngles).y;
        currentAngle6 = (float)(currentAngle6 * 180 / Math.PI);
        //Debug.Log("Gripper_rotation: " + currentAngle6);
        float Target_rotation = target.transform.localRotation.eulerAngles.y;

        Vector3 midpoint = ((transform.InverseTransformPoint(GripperA.transform.position) + transform.InverseTransformPoint(GripperB.transform.position))/2)+ upVector*0.008f; 
        Midpoint.transform.localPosition = midpoint;
        //Debug.Log("midpoint: " + midpoint);

        Debug.Log("distance: " + Vector3.Distance(midpoint, target.transform.position));
        Debug.Log(Link6.velocity.magnitude);
        Debug.Log("reward" + (-(Link6.velocity.magnitude - Vector3.Distance(midpoint, target.transform.position))));

        float angleDiff = Mathf.Abs(currentAngle6 - Target_rotation);
        while (angleDiff > 150)
        {
            angleDiff -= 180;
        }

        //Debug.Log("angleDiff: " + angleDiff);
        //Debug.Log("angleReward: " + CalculateReward(Gripper_angle, angleDiff, deviation));
        float CalculateReward(float Gripper_angle, float rotation_algle, float deviation)
        {
            // 
            float deviationFrom180 = Math.Abs(Gripper_angle - 180.0f);

            // 
            float penalty = (float)Math.Exp(-Math.Pow(deviationFrom180, 2) / (2 * Math.Pow(deviation, 2)));
            float penalty2 = (float)Math.Exp(-Math.Pow(rotation_algle, 2) / (2 * Math.Pow(deviation, 2)));

            // 
            return (penalty+penalty2);
        }
    }
}
