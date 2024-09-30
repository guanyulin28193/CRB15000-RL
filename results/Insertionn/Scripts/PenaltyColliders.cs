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
        //Debug.LogWarning("Penalty: " + gameObject.name + " collided with " + collision.gameObject.name);

        // Select agent
        if (agentInsertion != null)
        {
            if (gameObject.name == "FingerA" || gameObject.name == "FingerB" || gameObject.name == "Peg")
            {
                agentInsertion.PegHitPenalty( gameObject, collision.gameObject);
            }
            else
            {
                agentInsertion.GroundHitPenalty(gameObject, collision.gameObject);
            }
        }
        else if (platformAgent != null)
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
        else
        {
            Debug.LogWarning("No agent assigned to handle the penalty.");
        }
    }
}
