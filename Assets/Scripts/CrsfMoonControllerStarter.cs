using UnityEngine;

public class CrsfMoonControllerStarter : MonoBehaviour
{
    [SerializeField] private string m_ComPort = "COM5";
    [SerializeField] private int m_BaudRate = 420000;
    [SerializeField] private int m_SendRate = 20;
    [SerializeField] private CrsfMoonController m_CrsfMoonController;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        m_CrsfMoonController.Connect(m_ComPort, m_BaudRate, m_SendRate);
    }
}
