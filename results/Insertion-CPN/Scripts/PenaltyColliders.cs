using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents; 

public class PenaltyColliders: MonoBehaviour
{
    public AgentInsertion agentInsertion;
    public PlatformAgent platformAgent;
    public GraspVfAgent graspVfAgent;
    /*void Start()
    {
        Transform currentTransform = transform;

        while (currentTransform != null)
        {
            // 尝试从当前对象中获取组件
            agentInsertion = currentTransform.GetComponent<AgentInsertion>();
            platformAgent = currentTransform.GetComponent<PlatformAgent>();
            graspVfAgent = currentTransform.GetComponent<GraspVfAgent>();

            // 如果成功找到所有组件，则退出循环
            if (agentInsertion != null || platformAgent != null || graspVfAgent != null)
            {
                //Debug.Log("Successfully found components on " + currentTransform.name);
                break;
            }

            // 继续向上移动到父对象
            currentTransform = currentTransform.parent;
        }
    }*/

    private void OnCollisionEnter(Collision collision)
    {
        //Debug.LogWarning("Penalty: " + gameObject.name + " collided with " + collision.gameObject.name);
        if (gameObject.name == "tool0")
        {
            
        }
        // Select agent
        else if (agentInsertion != null && agentInsertion.enabled == true)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB" || gameObject.name == "Peg" || gameObject.name == "tool0")
            {
                agentInsertion.PegHitPenalty( gameObject, collision.gameObject);
            }
            else
            {
                agentInsertion.GroundHitPenalty(gameObject, collision.gameObject);
            }
        }
        else if (platformAgent != null && platformAgent.enabled == true)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB")
            {
                platformAgent.PegHitPenalty(gameObject, collision.gameObject);
            }
            else
            {
                platformAgent.GroundHitPenalty(gameObject, collision.gameObject);
            }
        }
        else if (graspVfAgent != null && graspVfAgent.enabled == true)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB")
            {
                graspVfAgent.PegHitPenalty(gameObject, collision.gameObject);
            }
            else
            {
                graspVfAgent.GroundHitPenalty(gameObject, collision.gameObject);
            }
        }
        else
        {
            Debug.LogWarning("No agent assigned to handle the enter penalty.");
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        // Select agent
        if (gameObject.name == "tool0")
        {
            
        }
        else if (agentInsertion != null && agentInsertion.enabled == true)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB" || gameObject.name == "Peg" || gameObject.name == "tool0")
            {
                agentInsertion.PegHitPenalty( gameObject, collision.gameObject);
            }
            else
            {
                agentInsertion.GroundHitPenalty(gameObject, collision.gameObject);
            }
        }
        else if (platformAgent != null && platformAgent.enabled == true)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB")
            {
                platformAgent.PegHitPenalty(gameObject, collision.gameObject);
            }
            else
            {
                platformAgent.GroundHitPenalty(gameObject, collision.gameObject);
            }
        }
        else if (graspVfAgent != null && graspVfAgent.enabled == true)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB")
            {
                graspVfAgent.PegHitPenalty(gameObject, collision.gameObject);
            }
            else
            {
                graspVfAgent.GroundHitPenalty(gameObject, collision.gameObject);
            }
        }
        else
        {
            Debug.LogWarning("No agent assigned to handle the Stay penalty.");
        }
    }
}
