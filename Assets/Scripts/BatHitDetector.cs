using UnityEngine;

public class BatHitDetector : MonoBehaviour
{
    public bool hasHitBall { get; private set; } = false;
    private BaseballAgent agent;

    private void Start()
    {
        agent = GetComponentInParent<BaseballAgent>();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ball") && !hasHitBall)
        {
            hasHitBall = true;
            agent.OnBallHit(collision);
        }
    }

    public void ResetHit()
    {
        hasHitBall = false;
    }
}
