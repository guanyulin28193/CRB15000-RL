using UnityEngine;
using Unity.MLAgents; 

public class BtTaskSwitcher: MonoBehaviour
{
    public AgentInsertion agentInsertion;
    public PlatformAgent platformAgent;
    public void Grasp_Success(Vector3 offset_from_link6)
    {
        Debug.Log("Grasp_Success, switching to insertion task with offset: " + offset_from_link6);
        //Disable the platform agent
        //Enable the agent insertion
        //Disable the behaviours parameter of the platform agent
        //Enable the behaviours parameter of the agent insertion
    }
    public void Grasp_failed()
    {
        Debug.Log("Grasp_failed, recording failure");
        //Recording the failure to Log
    }
    public void Insertion_Success()
    {
        Debug.Log("Insertion_Success, switching to grasp task");
        //Disable the agent insertion
        //Enable the platform agent
        //Disable the behaviours parameter of the agent insertion
        //Enable the behaviours parameter of the platform agent
    }
    public void Insertion_failed()
    {
        Debug.Log("Insertion_failed, recording failure");
        //Recording the failure to Log
    }
}