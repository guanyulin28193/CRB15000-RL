using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents; 

public class PenaltyColliders: MonoBehaviour
{
    public AgentInsertion agentInsertion;
    public PlatformAgent platformAgent;

    private void OnCollisionEnter(Collision collision)
    {
        Debug.LogWarning("Penalty: " + gameObject.name + " collided with " + collision.gameObject.name);

        // Select agent
        if (agentInsertion != null)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB")
            {
                agentInsertion.PegHitPenalty(collision.gameObject);
            }
            else
            {
                agentInsertion.GroundHitPenalty();
            }
        }
        else if (platformAgent != null)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB")
            {
                platformAgent.PegHitPenalty(collision.gameObject);
            }
            else
            {
                platformAgent.GroundHitPenalty();
            }
        }
        else
        {
            Debug.LogWarning("No agent assigned to handle the penalty.");
        }
    }
}
