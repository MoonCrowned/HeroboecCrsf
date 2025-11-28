using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class CrsfControllerTester : MonoBehaviour
{
    [SerializeField] private Slider[] m_ChannelSliders;
    [SerializeField] private TMP_Text m_TelemetryText;

    [SerializeField] private string m_CRSFport = "COM4";
    [SerializeField] private int m_CRSFbaud = 115200;
    [SerializeField] private int m_CRSFSendRate = 10;

    [SerializeField] private CrsfMoonController m_CrsfController;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        m_CrsfController.Connect(m_CRSFport, m_CRSFbaud, m_CRSFSendRate);
        m_CrsfController.TelemetryDataReceived += TelemetryDataReceived;
    }

    private void Update()
    {
        for (int i = 0; i < m_ChannelSliders.Length; i++)
        {
            m_CrsfController.SetChannel(i, m_ChannelSliders[i].value);
        }
    }

    private void TelemetryDataReceived(CrsfTelemetryData data)
    {
        m_TelemetryText.text = data.ToString();
    }
}
