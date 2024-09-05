using UnityEngine;
using Unity.MLAgents; 
using Unity.MLAgents.Policies;
using Unity.Barracuda;

public class BtTaskSwitcher: MonoBehaviour
{
    public BehaviorParameters InsertionBP;
    public BehaviorParameters behaviourParameters;
    public BehaviorParameters GraspBP;
    public bool SwitchtoInsertion;
    public NNModel InsertionModel;
    public NNModel GraspModel;
    public PlatformAgent platformAgent;
    public AgentInsertion agentInsertion;
    void Grasp_Success()
    {
        Debug.Log("Grasp_Success, switching to insertion task with offset: ");
        //pause the Unity
        //Disable the platform agent
        platformAgent.enabled = false;
        //Enable the agent insertion
        agentInsertion.enabled = true;
        //Disable the behaviours parameter of the platform agent
        behaviourParameters.Model = InsertionModel;
    }
    void Grasp_failed()
    {
        Debug.Log("Grasp_failed, recording failure");
        //Recording the failure to Log
    }
    void Insertion_Completed(bool success)
    {
        if (success)
        {
        Debug.Log("Insertion_Success, switching to grasp task");
        //Write Log
        }
        else
        {
        Debug.Log("Insertion_failed, switching to grasp task");
        //Disable the agent insertion
        //Enable the platform agent
        //Disable the behaviours parameter of the agent insertion
        //Enable the behaviours parameter of the platform agent
        }
    }
    void Start()
    {
        //Get the behavior parameters of the agent insertion
        //BehaviorParameters [] components = gameObject.GetComponents<BehaviorParameters>();
        behaviourParameters = gameObject.GetComponent<BehaviorParameters>();
        platformAgent = gameObject.GetComponent<PlatformAgent>();
        agentInsertion = gameObject.GetComponent<AgentInsertion>();
        /*foreach (BehaviorParameters component in components)
        {
            if (component.Model.name=="Insertion")
            {
                InsertionBP = component;
                Debug.Log("Successful bind the Insert Behaviour Parameters");
            }
            else if (component.Model.name=="Grasp")
            {
                //GraspBP = component;
                Debug.Log("Successful bind the Grasp Behaviour Parameters");
            }
        }*/
    }
    void FixedUpdate()
    {
        //Check if the agent is in the grasp task
        //Check if the agent is in the insertion task
        //Check if the agent is in the idle task
        if (SwitchtoInsertion)
        {
            Grasp_Success();
            SwitchtoInsertion = false;
        }
    }
}