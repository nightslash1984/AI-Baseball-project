using UnityEngine;

public class GroundDetect : MonoBehaviour
{
    public bool batOnGround { get; private set; } = false;
    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Ground"))
        {
            batOnGround = true;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Ground"))
        {
            batOnGround = false;
        }
    }
}
