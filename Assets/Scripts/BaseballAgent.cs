using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEditor;
using UnityEngine.InputSystem.HID;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using Unity.Collections;
using System;
using System.Linq;
using TMPro;

public class BodyPart
{
    public Transform transform;
    public Rigidbody rb;
    public CharacterJoint joint;

    public Vector3 initialLocalPosition;
    public Quaternion initialLocalRotation;

    public void Reset()
    {
        transform.localPosition = initialLocalPosition;
        transform.localRotation = initialLocalRotation;

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            rb.position = transform.position;
            rb.rotation = transform.rotation;
        }
    }

    public void ApplyTorque(Vector3 torque, float multiplier)
    {
        if (rb != null)
        {
            rb.AddRelativeTorque(torque * multiplier, ForceMode.Acceleration);
        }
    }
}

public class BaseballAgent : Agent
{
    [HideInInspector]
    public Bounds battersBox;

    [Header("Enviroment")]
    [SerializeField] private Transform spawnPosition;
    [SerializeField] private Transform bat;
    private Transform _ball;

    [Header("Training")]
    [SerializeField] int _currentEpisode = 0;
    private Dictionary<Transform, Rigidbody> bodyPartToRb = new Dictionary<Transform, Rigidbody>();
    private Rigidbody _ballRigidbody;
    [SerializeField] Rigidbody batRb;

    private float timeSinceEpisodeStart = 0f;
    [SerializeField] private float ballThrowDelay = 7f; // seconds
    bool ballThrown = false;
    [SerializeField] float launchForce = 0.0f;
    [SerializeField] GameObject ball;
    [SerializeField] Transform throwPos;

    [SerializeField] private float armSwingTorqueMultiplier = 500f;    // Right arm (bat arm)
    [SerializeField] private float armBalanceTorqueMultiplier = 300f;  // Left arm (balance arm)
    [SerializeField] private float bodyTorqueMultiplier = 200f;        // Core and legs

    private Rigidbody ballRb;

    // Rewards
    [SerializeField] private float contactReward = 1.0f;
    [SerializeField] private float distanceRewardMultiplier = 0.05f;
    [SerializeField] private float foulPenalty = 0.2f;
    [SerializeField] private float maxBallSpeed = 20f; // Max expected ball velocity for clamping


    private float standingTimer = 0f;
    [SerializeField] private float standingThreshold = 0.8f; // Minimum hip height to be considered standing
    [SerializeField] private float standingRewardFactor = 0.01f; // Base reward per second of standing
    private float batGroundedTimer = 0f;

    [SerializeField] float batGroundedPenalty = -0.001f;
    [SerializeField] GroundDetect groundCheck;

    [SerializeField] private Vector3 battersBoxCenter;
    [SerializeField] private Vector3 battersBoxSize;
    [SerializeField] private float boxExitPenalty = 0.01f;

    [SerializeField] private float leftFoulAngle = -45f;  // Degrees (left foul line)
    [SerializeField] private float rightFoulAngle = 45f;  // Degrees (right foul line)
    [SerializeField] private Transform homePlate;

    private bool waitingForPostHitReward = false;
    private float postHitTimer = 0f;
    private float postHitDelay = 1.5f; // Wait 1.5s after hit before evaluating

    private Vector3 ballHitVelocity;

    [SerializeField] private float missedBallPenalty = 1.0f;

    [SerializeField] BatHitDetector batHitDetector;

    [SerializeField] private BallWallHitDetector ballWallHitDetector;

    [SerializeField] private TMP_Text rewardText;

    [Header("Body Parts")]
    [SerializeField] private Transform[] bodyPartTransforms;

    private List<BodyPart> bodyParts = new List<BodyPart>();
    /*
    [SerializeField] Transform _spine;     0
    [SerializeField] Transform _head;      1
    [SerializeField] Transform _thighL;    2
    [SerializeField] Transform _shinL;     3
    [SerializeField] Transform _footL;     4
    [SerializeField] Transform _thighR;    5
    [SerializeField] Transform _shinR;     6
    [SerializeField] Transform _footR;     7
    [SerializeField] Transform _armL;      8
    [SerializeField] Transform _forearmL;  9
    [SerializeField] Transform _handL;     10
    [SerializeField] Transform _armR;      11
    [SerializeField] Transform _forearmR;  12
    [SerializeField] Transform _handR;     13
    [SerializeField] Transform _hips;      14
    */

    private void Start()
    {
        battersBox = new Bounds(battersBoxCenter, battersBoxSize);
    }
    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(battersBoxCenter, battersBoxSize);
        Gizmos.color = Color.red;

        Vector3 startPoint = homePlate.localPosition + new Vector3(0, 0.352f, 0); ; // Assume batter stands at agent position
        float lineLength = 100f; // Length of the foul lines

        // Calculate foul line directions
        Vector3 leftFoulDirection = Quaternion.Euler(0f, leftFoulAngle, 0f) * -Vector3.forward;
        Vector3 rightFoulDirection = Quaternion.Euler(0f, rightFoulAngle, 0f) * -Vector3.forward;

        // Draw lines
        Gizmos.DrawLine(startPoint, startPoint + leftFoulDirection * lineLength);
        Gizmos.DrawLine(startPoint, startPoint + rightFoulDirection * lineLength);
    }

    public override void Initialize()
    {
        Debug.Log("Initalize()");

        foreach (var t in bodyPartTransforms)
        {
            var part = new BodyPart
            {
                transform = t,
                rb = t.GetComponent<Rigidbody>(),
                joint = t.GetComponent<CharacterJoint>(),
                initialLocalPosition = t.localPosition,
                initialLocalRotation = t.localRotation
            };

            bodyParts.Add(part);
        }

        _currentEpisode = 0;
    }

    public override void OnEpisodeBegin()
    {
        //Debug.Log("OnEpisodeBegin() " + _currentEpisode);
        timeSinceEpisodeStart = 0f;
        batHitDetector.ResetHit();
        waitingForPostHitReward = false;
        postHitTimer = 0f;
        ballThrown = false;
        standingTimer = 0f;

        foreach (var part in bodyParts)
        {
            part.Reset();
        }

        _currentEpisode++;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        rewardText.text = GetCumulativeReward().ToString();

        // Agent body parts
        foreach (var part in bodyParts)
        {
            // Local position
            Vector3 localPos = transform.InverseTransformPoint(part.transform.position);
            sensor.AddObservation(localPos);

            // Local velocity
            if (part.rb != null)
            {
                Vector3 localVel = transform.InverseTransformDirection(part.rb.linearVelocity);
                sensor.AddObservation(localVel);

                Vector3 localAngVel = transform.InverseTransformDirection(part.rb.angularVelocity);
                sensor.AddObservation(localAngVel);
            }
            else
            {
                // Fallback if no Rigidbody (unlikely but safe)
                sensor.AddObservation(Vector3.zero);
                sensor.AddObservation(Vector3.zero);
            }
        }

        // Ball observations
        if (_ball != null && _ballRigidbody != null)
        {
            Vector3 ballRelativePos = transform.InverseTransformPoint(_ball.position);
            sensor.AddObservation(ballRelativePos);

            Vector3 ballVelocityLocal = transform.InverseTransformDirection(_ballRigidbody.linearVelocity);
            sensor.AddObservation(ballVelocityLocal);
        }

        // Bat observations
        if (bat != null && batRb != null)
        {
            sensor.AddObservation(transform.InverseTransformPoint(bat.position));
            sensor.AddObservation(transform.InverseTransformDirection(batRb.linearVelocity));
            sensor.AddObservation(transform.InverseTransformDirection(batRb.angularVelocity));
        }

        // Uprightness (dot with world up)
        sensor.AddObservation(Vector3.Dot(transform.up, Vector3.up));

        // Hip height
        var hipPart = bodyParts.FirstOrDefault(bp => bp.transform.name.ToLower().Contains("b-hips"));
        float hipHeight = hipPart != null ? hipPart.transform.position.y : 0f;
        sensor.AddObservation(hipHeight);
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        timeSinceEpisodeStart += Time.deltaTime;
        int actionIndex = 0;

        for (int i = 0; i < bodyParts.Count; i++)
        {
            var part = bodyParts[i];

            if (part.rb != null)
            {
                float torqueX = actions.ContinuousActions[actionIndex++];
                float torqueY = actions.ContinuousActions[actionIndex++];
                float torqueZ = actions.ContinuousActions[actionIndex++];

                Vector3 torque = new Vector3(torqueX, torqueY, torqueZ);

                // Identify part by name and apply proper torque multiplier
                string name = part.transform.name.ToLower();

                if (name.Contains("b-upperarm.r") || name.Contains("b-forearm.r") || name.Contains("b-handprop.r"))
                {
                    torque *= armSwingTorqueMultiplier;
                }
                else if (name.Contains("b-upperarm.l") || name.Contains("b-forearm.l") || name.Contains("b-handprop.l"))
                {
                    torque *= armBalanceTorqueMultiplier;
                }
                else
                {
                    torque *= bodyTorqueMultiplier;
                }

                part.rb.AddRelativeTorque(torque, ForceMode.Acceleration);
            }
        }

        RewardStanding();
        CheckFall();
        PenalizeBatTouchingGround();
        PenalizeLeavingBattersBox();

        // Check if it's time to throw the ball
        if (!ballThrown && timeSinceEpisodeStart >= ballThrowDelay)
        {
            ThrowBall();
            ballThrown = true;
        }

        // Post-hit evaluation
        if (waitingForPostHitReward)
        {
            postHitTimer += Time.deltaTime;
            if (postHitTimer >= postHitDelay)
            {
                EvaluateHit(ballHitVelocity);
                waitingForPostHitReward = false;
                EndEpisode();
            }
        }
    }

    private void EvaluateHit(Vector3 velocity)
    {
        Vector3 horizontalVelocity = new Vector3(velocity.x, 0f, velocity.z);

        if (horizontalVelocity.sqrMagnitude < 0.1f)
            return; // Ball didn't go far

        // Calculate angle
        float angle = Vector3.SignedAngle(Vector3.forward, horizontalVelocity.normalized, Vector3.up);
        bool isFair = angle >= leftFoulAngle && angle <= rightFoulAngle;

        if (isFair)
        {
            float baseDistanceReward = Mathf.Clamp(horizontalVelocity.magnitude, 0f, maxBallSpeed) * distanceRewardMultiplier;

            if (ballWallHitDetector.hasHitWall)
            {
                // Give full reward — wall stopped it
                AddReward(baseDistanceReward);
                Debug.Log("Hit wall in fair territory — rewarded fully.");
            }
            else
            {
                // Use raw distance-based reward
                AddReward(baseDistanceReward);
            }
        }
        else
        {
            AddReward(-foulPenalty);
        }
    }


    private void ThrowBall()
    {
        GameObject launchedBall = Instantiate(ball, throwPos.position, Quaternion.identity);
        _ball = launchedBall.transform;
        _ballRigidbody = launchedBall.GetComponent<Rigidbody>();
        _ballRigidbody.AddForce(new Vector3(0.0f, 0.0f, launchForce));
        ballWallHitDetector = launchedBall.GetComponent<BallWallHitDetector>();
        ballRb = _ballRigidbody; // Keep both updated

        #if UNITY_EDITOR
        UnityEditor.Selection.activeObject = launchedBall;
        #endif
    }

    private void RewardStanding()
    {
        // How upright the agent is (dot product)
        float uprightBonus = Mathf.Clamp01(Vector3.Dot(transform.up, Vector3.up));
        var hipPart = bodyParts.FirstOrDefault(bp => bp.transform.name.ToLower().Contains("b-hips"));

        // Measure hip height
        float hipHeight = hipPart.transform.localPosition.y;

        if (hipHeight > standingThreshold)
        {
            standingTimer += Time.deltaTime; // Count how long standing
            float reward = uprightBonus * standingRewardFactor * standingTimer; // Reward grows over time
            AddReward(reward);
        }
        else
        {
            // If agent falls too low, reset standing timer
            standingTimer = 0f;
        }
    }
    private void CheckFall()
    {
        var hipPart = bodyParts.FirstOrDefault(bp => bp.transform.name.ToLower().Contains("b-hips"));
        //Debug.Log(hipPart.transform.position.y);
        if (hipPart.transform.position.y < 0.3f) // Too close to ground = fallen
        {
            Debug.Log("Falling over");
            AddReward(-1f); // Big penalty
            EndEpisode(); // Reset
        }
    }

    private void PenalizeBatTouchingGround()
    {
        if (groundCheck.batOnGround)
        {
            Debug.Log("Bat Touched Ground");
            AddReward(batGroundedPenalty);
            EndEpisode();
        }
    }
    public void OnBallMissed()
    {
        if (!batHitDetector.hasHitBall && ballThrown)
        {
            Debug.Log("Missed Ball");
            AddReward(-missedBallPenalty);
            EndEpisode();
        }
        Debug.Log("Agent missed the ball — episode ended.");
    }


    private void PenalizeLeavingBattersBox()
    {
        if (!battersBox.Contains(transform.localPosition))
        {
            Debug.Log("Left Batters Box");
            AddReward(-boxExitPenalty);
            EndEpisode();
        }
    }

    public void OnBallHit(Collision collision)
    {
        AddReward(contactReward); // Immediate reward for contact

        // Store ball's velocity at moment of contact
        ballHitVelocity = ballRb.linearVelocity;

        // Start post-hit evaluation
        postHitTimer = 0f;
        waitingForPostHitReward = true;
    }


}
