using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class MinionAgent : Agent
{
    [SerializeField] private Rigidbody rb;
    [SerializeField] private float moveForce = 10f;
    [SerializeField] private float rewardMultiplier = 0.05f;

    private Vector3 lastPosition;

    public override void Initialize()
    {
        if (rb == null)
        {
            rb = GetComponent<Rigidbody>();
        }

        rb.drag = 0.2f;
        rb.angularDrag = 0.5f;
        rb.centerOfMass = new Vector3(0f, -0.5f, 0f); // Lower center of mass
    }

    public override void OnEpisodeBegin()
    {
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.localPosition = new Vector3(0f, 0.5f, 0f);
        transform.rotation = Quaternion.identity;

        lastPosition = transform.localPosition;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);      // 3
        sensor.AddObservation(rb.velocity);                 // 3
        sensor.AddObservation(transform.up);                // 3 (uprightness)
        sensor.AddObservation(transform.forward);           // 3 (direction)
        sensor.AddObservation(rb.angularVelocity);          // 3

        // Total = 15
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        Vector3 force = new Vector3(moveX, 0f, moveZ);
        rb.AddForce(force * moveForce);

        // ➕ Reward: Forward Movement
        float distanceMoved = Vector3.Distance(transform.localPosition, lastPosition);
        AddReward(distanceMoved * rewardMultiplier);
        lastPosition = transform.localPosition;

        // ➕ Reward: Uprightness
        float uprightness = Vector3.Dot(transform.up, Vector3.up); // 0 to 1
        AddReward(uprightness * 0.05f);

        // ➖ Penalty: Spinning
        AddReward(-rb.angularVelocity.magnitude * 0.01f);

        // ❌ Fell off
        if (transform.localPosition.y < 0.1f)
        {
            SetReward(-1f);
            EndEpisode();
        }

        // ✅ Reached goal
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
