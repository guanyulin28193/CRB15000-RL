using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PenaltyColliders : MonoBehaviour
{
    public PlatformAgent agent;

    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Penalty: " + gameObject.name + " collided with " + collision.gameObject.name);

        if (gameObject.name == "link_6" || gameObject.name == "flange")
        {
            
        }
        else
        {
            agent.GroundHitPenalty();
        }

        if (gameObject.name == "FingerA" || gameObject.name == "FingerB")
        {
            agent.PegHitPenalty(collision.gameObject);
        }
        else
        {
            agent.GroundHitPenalty();
        }
    }
}
