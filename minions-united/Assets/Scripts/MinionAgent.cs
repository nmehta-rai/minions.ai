using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections;

public class MinionAgent : Agent
{
    [SerializeField] private Rigidbody rb;
    private Vector3 lastPosition;

    public override void Initialize()
    {
        if (rb == null)
        {
            rb = GetComponent<Rigidbody>();
        }

        rb.drag = 0.2f;
        rb.angularDrag = 0.5f;
    }

    public override void OnEpisodeBegin()
    {
        StartCoroutine(ResetAgent());
    }

    private IEnumerator ResetAgent()
    {
        yield return new WaitForSeconds(0.25f);

        transform.localPosition = new Vector3(0f, 0.5f, 0f);
        transform.rotation = Quaternion.identity;

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        lastPosition = transform.localPosition;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);    // 3
        sensor.AddObservation(rb.velocity);               // 3
        sensor.AddObservation(transform.up);              // 3
        sensor.AddObservation(transform.forward);         // 3
        sensor.AddObservation(rb.angularVelocity);        // 3

        // Total = 15
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];  // sideways
        float moveZ = actions.ContinuousActions[1];  // forward/back

        Vector3 force = new Vector3(moveX, 0f, moveZ);
        rb.AddForce(force * 10f);  // 🚀 Boosted force

        // Encourage movement
        float distanceMoved = Vector3.Distance(transform.localPosition, lastPosition);
        AddReward(distanceMoved * 0.1f);
        lastPosition = transform.localPosition;

        // Gentle tipping penalty
        if (transform.up.y < 0.6f)
        {
            AddReward(-0.05f);
        }

        // Fell off
        if (transform.localPosition.y < 0.1f)
        {
            SetReward(-1f);
            EndEpisode();
        }

        // Win condition
        if (transform.localPosition.z > 5f)
        {
            SetReward(+1f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
