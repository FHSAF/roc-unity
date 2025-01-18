using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }

    [SerializeField]
    List<GameObject> m_Targets = new List<GameObject>();
    int m_CurrentTargetIndex = 0;

    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    [SerializeField]
    float m_PlaceHeightIncrement = 0.02f; // Height increment for each placed item

    float m_CurrentPlaceHeight = 0.01f; // Current stacking height

    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    ROSConnection m_Ros;

    // START: New fields for MQTT integration and resume functionality
    bool isHalted = false;
    bool shouldReturnToStart = false;
    NiryoMoveitJointsMsg initialJointConfig;

    MoverServiceResponse haltedResponse;
    int haltedPoseIndex = -1;
    int haltedTrajectoryIndex = -1;
    bool shouldResume = false;

    Coroutine trajectoryCoroutine;
    // END: New fields for MQTT integration and resume functionality

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        if (m_Targets.Count == 0)
        {
            m_Targets = GameObject.FindGameObjectsWithTag("Target").ToList();
            if (m_Targets.Count == 0)
            {
                Debug.LogError("No targets found with tag 'Target'.");
            }
        }

        // START: New code for saving initial joint configuration and subscribing to MQTT
        initialJointConfig = CurrentJointConfig();
        MQTTHandler.OnMessageReceived += HandleMqttMessage;
        // END: New code for saving initial joint configuration and subscribing to MQTT
    }

    void OnDestroy()
    {
        // START: Unsubscribe from MQTT events
        MQTTHandler.OnMessageReceived -= HandleMqttMessage;
        // END: Unsubscribe from MQTT events
    }

    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.015f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.015f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }
        Debug.Log("Current Joint State: " + string.Join(", ", joints.joints));
        return joints;
    }

    public void PublishJoints()
    {
        if (m_Targets.Count == 0)
        {
            Debug.LogError("No targets available.");
            return;
        }

        var m_Target = m_Targets[Mathf.Min(m_CurrentTargetIndex, m_Targets.Count - 1)];
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Assembly Pose (hardcoded joint values)
        request.assembly_pose = new NiryoMoveitJointsMsg
        {
            joints = new[] {
                            -2.69129848480225f,
                            -0.446857243776321f,
                            0.0093951104208827f,
                            3.62700120604131E-05f,
                            0.408536493778229f,
                            7.95792161056852E-08f
                        }.Select(value => (double)value).ToArray()
        };

        // Place Pose
        var placementPosition = m_TargetPlacement.transform.position + m_PickPoseOffset;
        placementPosition.y += m_CurrentPlaceHeight; // Increment height for stacking
        request.place_pose = new PoseMsg
        {
            position = placementPosition.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_CurrentTargetIndex++;
        m_CurrentPlaceHeight += m_PlaceHeightIncrement; // Update the stacking height for the next item
        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            if (trajectoryCoroutine != null)
            {
                Debug.Log("Stopping existing trajectory coroutine...");
                StopCoroutine(trajectoryCoroutine);
            }
            trajectoryCoroutine = StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories == null)
        {
            Debug.LogError("No trajectories received. Execution aborted.");
            yield break;
        }

        Debug.Log($"Starting trajectory execution. Halted={isHalted}, ShouldResume={shouldResume}");

        for (var poseIndex = (shouldResume ? haltedPoseIndex : 0); poseIndex < response.trajectories.Length; poseIndex++)
        {
            for (var trajIndex = (shouldResume && poseIndex == haltedPoseIndex ? haltedTrajectoryIndex : 0);
                trajIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; trajIndex++)
            {
                // Check for halt condition and pause execution until resumed
                while (isHalted)
                {
                    Debug.Log($"Execution halted at poseIndex={poseIndex}, trajIndex={trajIndex}");
                    haltedPoseIndex = poseIndex;
                    haltedTrajectoryIndex = trajIndex;
                    haltedResponse = response; // Save response for resumption

                    yield return null; // Wait for the next frame while the robot is halted
                }

                // Reset resume flags after resuming
                shouldResume = false;

                // Apply joint positions
                var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[trajIndex].positions;
                var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                {
                    var jointXDrive = m_JointArticulationBodies[joint].xDrive;
                    jointXDrive.target = result[joint];
                    m_JointArticulationBodies[joint].xDrive = jointXDrive;
                }

                yield return new WaitForSeconds(k_JointAssignmentWait);
            }

            // Perform specific actions at poses
            if (poseIndex == (int)Poses.Grasp)
            {
                CloseGripper();
            }
            else if (poseIndex == (int)Poses.Assembly)
            {
                Debug.Log("Reached Assembly Pose. Waiting for 10 seconds...");
                yield return new WaitForSeconds(10.0f); // Wait for 10 seconds at assembly pose
            }
            else if (poseIndex == (int)Poses.Place)
            {
                Debug.Log("Item placed. Raising placement height.");
                // Perform any additional actions for placing
            }

            yield return new WaitForSeconds(k_PoseAssignmentWait);
        }

        // Reset halt state after successful execution
        Debug.Log("Execution completed. Resetting halt state.");
        haltedPoseIndex = -1;
        haltedTrajectoryIndex = -1;
        haltedResponse = null;

        OpenGripper(); // Ensure the gripper is open at the end
    }
    
    // START: New method for handling MQTT messages
    void HandleMqttMessage(string message)
    {
        switch (message)
        {
            case "halt":
                isHalted = true;
                Debug.Log("Halting execution...");
                break;

            case "resume":
                Debug.Log($"Attempting to resume: isHalted={isHalted}, haltedResponse={(haltedResponse != null)}");
                if (isHalted && haltedResponse != null)
                {
                    isHalted = false;
                    shouldResume = true;

                    if (trajectoryCoroutine != null)
                    {
                        Debug.Log("Stopping existing trajectory coroutine...");
                        StopCoroutine(trajectoryCoroutine);
                    }

                    Debug.Log("Resuming execution...");
                    trajectoryCoroutine = StartCoroutine(ExecuteTrajectories(haltedResponse)); // Resume forward execution
                }
                else
                {
                    Debug.LogWarning("Cannot resume. Either not halted or no trajectory saved.");
                }
                break;

            case "return":
                Debug.Log("Return requested. Executing reverse trajectory.");

                // Reset relevant flags for return
                isHalted = false;
                shouldResume = true;

                // Stop any existing trajectory coroutine if it's running
                if (trajectoryCoroutine != null)
                {
                    Debug.Log("Stopping existing trajectory coroutine...");
                    StopCoroutine(trajectoryCoroutine);
                }

                if (isHalted && haltedResponse != null)
                {
                    Debug.Log("Starting reverse trajectory execution...");
                    trajectoryCoroutine = StartCoroutine(ExecuteTrajectories(haltedResponse)); // Start reverse trajectory execution
                }
                else
                {
                    Debug.LogWarning("Cannot return. The robot is not halted or no trajectory exists.");
                }
                break;

            default:
                Debug.LogWarning($"Unknown command: {message}");
                break;
        }
    }




    // END: New method for handling MQTT messages

    // START: New method for returning to start position
    IEnumerator ReturnToStartPosition()
    {
        Debug.Log("Returning to start position...");
        for (int i = 0; i < initialJointConfig.joints.Length; i++)
        {
            var jointXDrive = m_JointArticulationBodies[i].xDrive;
            jointXDrive.target = (float)initialJointConfig.joints[i] * Mathf.Rad2Deg;
            m_JointArticulationBodies[i].xDrive = jointXDrive;
        }

        yield return new WaitForSeconds(k_PoseAssignmentWait);

        Debug.Log("Returned to start position. Resetting state.");
        shouldReturnToStart = false; // Reset flag after returning
        isHalted = false; // Reset halt state
    }


    // END: New method for returning to start position

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Assembly,
        Place
    }
}
