using UnityEngine;

public class BallWallHitDetector : MonoBehaviour
{
    public bool hasHitWall { get; private set; } = false;

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.root.CompareTag("OutfieldWall"))
        {
            hasHitWall = true;
        }
    }

    public void ResetWallHit()
    {
        hasHitWall = false;
    }
}
