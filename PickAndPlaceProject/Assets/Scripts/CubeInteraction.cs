using UnityEngine;

public class CubeInteraction : MonoBehaviour
{
    public GameObject snapPoint; // Assign this in the Inspector
    private bool isInstalled = false;
    void OnTriggerStay(Collider other)
    {
        Debug.Log("Installation Step2!");
        if (other.CompareTag("Tool") && !isInstalled)
        {
            Debug.Log("Installation Step3!");
            // Check if the tool is near the snap point
            float distance = Vector3.Distance(snapPoint.transform.position, other.transform.position);
            if (distance < 0.1f) // Adjust as needed
            {
                PerformInstallation();
            }
        }
    }

    void PerformInstallation()
    {
        isInstalled = true;
        Debug.Log("Installation Complete!");
        snapPoint.GetComponent<Renderer>().material.color = Color.green; // Visual feedback
    }
}
