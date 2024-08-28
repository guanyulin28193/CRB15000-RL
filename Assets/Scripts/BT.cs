using UnityEngine;
using Unity.MLAgents; 

public class BtTaskSwitcher: MonoBehaviour
{
    public AgentInsertion agentInsertion;
    public PlatformAgent platformAgent;
    public void Grasp_Success(Vector3 offset_from_link6)
    {
        Debug.Log("Grasp_Success, switching to insertion task with offset: " + offset_from_link6);
    }
}