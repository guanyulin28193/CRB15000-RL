using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PenaltyColliders : MonoBehaviour
{
    public LowLvlAgent agent;
    private void OnTriggerEnter(Collider other)
    {
        
        Debug.Log("Penalty: " + gameObject.name + " collided with " + other.gameObject.name);
        
        if ((gameObject.name == "FingerA" || gameObject.name == "FingerB") )
        {
            agent.PegHitPenalty(other.gameObject);
        }
        else
        {   
            agent.GroundHitPenalty();
        }
        
    }
}