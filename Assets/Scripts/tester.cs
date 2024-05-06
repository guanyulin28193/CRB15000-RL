using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class tester : MonoBehaviour
{
    public ArticulationBody GripperA;
    public ArticulationBody GripperB;
    public ArticulationBody Link2;
    public ArticulationBody Link3;
    public ArticulationBody Link4;
    public ArticulationBody Link5;
    public ArticulationBody Link6;
    public Transform target;

    void Update()
    {
        // 计算GripperA和GripperB的中点位置
        Vector3 midpoint = (GripperA.transform.position + GripperB.transform.position) / 2;

        // 计算目标位置与中点位置之间的距离
        var distanceToTarget = Vector3.Distance(target.position, midpoint);

        // 访问每个ArticulationBody的ArticulationDrive，并读取当前位置
        float currentAngleA = GripperA.jointPosition[0];
        float currentAngleB = GripperB.jointPosition[0];
        float currentAngle2 = Link2.jointPosition[0];
        float currentAngle3 = Link3.jointPosition[0];
        float currentAngle4 = Link4.jointPosition[0];
        float currentAngle5 = Link5.jointPosition[0];
        float currentAngle6 = Link6.jointPosition[0];
        var link2angleb = transform.InverseTransformDirection(Link2.transform.localRotation.eulerAngles);

        // 打印当前角度
        Debug.Log("GripperA当前角度: " + currentAngleA);
        Debug.Log("GripperB当前角度: " + currentAngleB);
        Debug.Log("Link2当前角度: " + currentAngle2);
        Debug.Log("Link2当前角度b: " + link2angleb);
        Debug.Log("Link3当前角度: " + currentAngle3);
        Debug.Log("Link4当前角度: " + currentAngle4);
        Debug.Log("Link5当前角度: " + currentAngle5);
        Debug.Log("Link6当前角度: " + currentAngle6);
    }
}
