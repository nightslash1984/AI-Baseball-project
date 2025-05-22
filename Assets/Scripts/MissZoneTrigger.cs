using UnityEngine;

public class MissZoneTrigger : MonoBehaviour
{
    private BaseballAgent agent;

    private void Start()
    {
        agent = GetComponentInParent<BaseballAgent>();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Ball"))
        {
            agent.OnBallMissed();
        }
    }
}
