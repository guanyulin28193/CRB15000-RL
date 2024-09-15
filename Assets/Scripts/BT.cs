using UnityEngine;
using Unity.MLAgents; 
using Unity.MLAgents.Policies;
using Unity.Barracuda;
using System.IO;

public class BtTaskSwitcher: MonoBehaviour
{
    public bool SwitchtoInsertion;
    public bool SwitchtoGrasp;
    public GameObject Grasp;
    public GameObject Insertion;
    public GameObject Peg;
    private Rigidbody PegRb;
    private AgentInsertion agentInsertion;
    private PlatformAgent platformAgent;
    private Vector3 BTOffset;
    private float [] Init_Angles;
    private int NextFrameInt = 0;
    private int FramesToWait = 7;
    
    //for logging data
    private string offsetString;
    private string anglesString;
    private static StreamWriter logWriter;
    private static string logFilePath;


    void Start()
    {
        agentInsertion = Insertion.GetComponent<AgentInsertion>();
        platformAgent = Grasp.GetComponent<PlatformAgent>();
        PegRb = Peg.GetComponent<Rigidbody>();
        InitializeLogFile();
    }
    void OnDisable()
    {
        CloseLogFile();
    }
    private void InitializeLogFile()
    {
        string directoryPath = "Assets/Logs";
        logFilePath = Path.Combine(directoryPath, "log.csv");

        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }

        bool fileExists = File.Exists(logFilePath);
        logWriter = new StreamWriter(logFilePath, true);

        if (!fileExists)
        {
            logWriter.WriteLine("Timestamp,BTOffset,Init_Angles,CumulativeReward,checkpointVisitedTimes,Vaild_CP,First_CP_Step");
        }
    }

    private void CloseLogFile()
    {
        if (logWriter != null)
        {
            logWriter.Flush();
            logWriter.Close();
            logWriter = null;
        }
    }
    void WriteFailLog()
    {
        logWriter.WriteLine(
            $"{System.DateTime.Now:yyyy-MM-dd HH:mm:ss}," +
            $"\"0,0,0\"," +  // BTOffset
            $"\"0,0,0,0,0,0\"," +  // Init_Angles
            $"0," +  // CumulativeReward
            $"\"0\"," +  // checkpointVisitedTimes
            $"0," +  // Vaild_CP
            $"0"  // First_CP_Step
        );
        logWriter.Flush();
    }
    void WriteSuccessLog(float CumulativeReward, int[] checkpointVisitedTimes, int Vaild_CP, int First_CP_Step)
    {
        string checkpointVisitedTimesString = string.Join(",", checkpointVisitedTimes);
        logWriter.WriteLine(
            $"{System.DateTime.Now:yyyy-MM-dd HH:mm:ss}," +
            $"\"{offsetString}\"," +
            $"\"{anglesString}\"," +
            $"{CumulativeReward}," +
            $"\"{checkpointVisitedTimesString}\"," +
            $"{Vaild_CP}," +
            $"{First_CP_Step}"
        );
        logWriter.Flush();
    }
    void fSwitchToInsertion(Vector3 BTOffset, float [] Init_Angles)
    {
        if (NextFrameInt == 0)
        {
            agentInsertion.BTOffset = BTOffset;
            agentInsertion.Init_Angles = Init_Angles;
            platformAgent.ResetAllAB();
            platformAgent.Resetter();
            platformAgent.EndEpisode();
            platformAgent.enabled = false;
        }

        if (NextFrameInt<FramesToWait)
        {
            platformAgent.ResetAllAB();
            platformAgent.Resetter();
            NextFrameInt++;
        }
        else
        {
            Grasp.SetActive(false);
            PegRb.useGravity = false;
            agentInsertion.enabled = true;
            Insertion.SetActive(true);
            NextFrameInt = 0;
            SwitchtoInsertion = false;
        }

    }
    void fSwitchToGrasp()
    {
        if (NextFrameInt == 0)
        {
            agentInsertion.DestroyJoint();
            agentInsertion.ResetAllAB();
            agentInsertion.Resetter();
            agentInsertion.EndEpisode();
            agentInsertion.enabled = false;
        }
        if (NextFrameInt<FramesToWait)
        {
            agentInsertion.DestroyJoint();
            agentInsertion.ResetAllAB();
            agentInsertion.Resetter();
            NextFrameInt++;
        }
        else
        {
            Insertion.SetActive(false);
            platformAgent.enabled = true;
            PegRb.useGravity = true;
            Grasp.SetActive(true);
            NextFrameInt = 0;
            SwitchtoGrasp = false;
        }
    }
    void FixedUpdate()
    {   
        if (platformAgent.HasSuccessfullyGrasped())
        {
            BTOffset = platformAgent.GetGraspOffset();
            Init_Angles = platformAgent.GetJointAngles();
            Debug.Log("Grasp_Success, switching to insertion task with offset: " + BTOffset);
            Debug.Log("Initial Angles: " + string.Join(" ", Init_Angles));
            SwitchtoInsertion = true;

            // Data to string for logging
            offsetString = string.Format("{0},{1},{2}", BTOffset.x, BTOffset.y, BTOffset.z);
            anglesString = string.Join(",", Init_Angles);
        }
        if (platformAgent.HasFailedGrasp())
        {
            Debug.Log("Grasp_Fail, try next episode");
            platformAgent.Resetter();
            WriteFailLog();
        }
        if (SwitchtoInsertion)
        {
            fSwitchToInsertion(BTOffset, Init_Angles);
        }
        if (agentInsertion.fInsertionComplete())
        {
            Debug.Log("Insertion completed, Switching to grasp task");

            float CumulativeReward = agentInsertion.GetCumulativeRewardexternal();
            int[] checkpointVisitedTimes = agentInsertion.GetcheckpointVisitedTimes();
            int Vaild_CP = agentInsertion.GetVaildCP();
            int First_CP_Step = agentInsertion.GetFirstCPStep();

            WriteSuccessLog(CumulativeReward, checkpointVisitedTimes, Vaild_CP, First_CP_Step);
            SwitchtoGrasp = true;
        }

        if (SwitchtoGrasp)
        {
            fSwitchToGrasp();
        }
    }
}