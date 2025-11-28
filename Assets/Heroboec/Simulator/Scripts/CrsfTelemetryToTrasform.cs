using UnityEngine;

public class CrsfTelemetryToTrasform : MonoBehaviour
{
    [SerializeField] private CrsfMoonController m_CrsfMoonController;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        m_CrsfMoonController.TelemetryDataReceived += TelemetryReceived;
    }

    private void TelemetryReceived(CrsfTelemetryData data)
    {
        transform.localEulerAngles = new Vector3(data.Angles.Pitch, data.Angles.Yaw, -data.Angles.Roll);
    }
}
