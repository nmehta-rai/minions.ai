using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class MinionAgentV2 : Agent
{
    [System.Serializable]
    public class JointControl
    {
        public string name;
        public Transform joint;
        public Rigidbody rb;
        public ConfigurableJoint configJoint;
        public Vector3 targetRotation;
    }

    public List<JointControl> jointControls;
    public Transform hips;
    public Transform target;  // Optional: goal to walk toward
    private Vector3 startPosition;

    public override void Initialize()
    {
        startPosition = hips.position;
    }

    public override void OnEpisodeBegin()
    {
        foreach (var jc in jointControls)
        {
            jc.rb.velocity = Vector3.zero;
            jc.rb.angularVelocity = Vector3.zero;
            jc.joint.localRotation = Quaternion.identity;
        }

        hips.position = startPosition + new Vector3(0, 0.5f, 0);
        hips.GetComponent<Rigidbody>().velocity = Vector3.zero;
        hips.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Hips state
        sensor.AddObservation(hips.forward);
        sensor.AddObservation(hips.up);
        sensor.AddObservation(hips.GetComponent<Rigidbody>().velocity);
        sensor.AddObservation(hips.GetComponent<Rigidbody>().angularVelocity);

        // Joints
        foreach (var jc in jointControls)
        {
            sensor.AddObservation(jc.joint.localRotation);
            sensor.AddObservation(jc.rb.angularVelocity);
        }

        // Optional: goal direction
        if (target != null)
        {
            sensor.AddObservation(target.position - hips.position);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int i = 0;

        foreach (var jc in jointControls)
        {
            Vector3 target = new Vector3(
                Mathf.Clamp(actions.ContinuousActions[i++], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[i++], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[i++], -1f, 1f)
            );

            jc.rb.AddTorque(target * 300f);
            Debug.DrawRay(jc.joint.position, target * 0.2f, Color.red);
        }

        // ==== REWARD SHAPING ====

        Rigidbody hipRb = hips.GetComponent<Rigidbody>();
        Vector3 localVelocity = hips.InverseTransformDirection(hipRb.velocity);

        float forwardSpeed = localVelocity.z;
        float sidewaysSpeed = Mathf.Abs(localVelocity.x);
        float verticalSpeed = Mathf.Abs(localVelocity.y);

        float uprightness = Vector3.Dot(hips.up, Vector3.up);

        AddReward(forwardSpeed * 0.2f);                  // encourage forward
        AddReward(-sidewaysSpeed * 0.05f);               // discourage wobble
        AddReward(-verticalSpeed * 0.02f);               // discourage hopping
        AddReward(uprightness * 0.05f);                  // stay upright

        // End episode if fallen
        if (hips.position.y < 0.3f || uprightness < 0.3f)
        {
            AddReward(-2f);
            EndEpisode();
        }

        // DEBUG
        Debug.Log($"→ F: {forwardSpeed:F2} | Side: {sidewaysSpeed:F2} | Up: {uprightness:F2}");
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual override (optional later)
    }
}
