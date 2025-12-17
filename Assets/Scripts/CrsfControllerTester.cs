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

    [SerializeField] private int m_RollChannel = 0;
    [SerializeField] private int m_PitchChannel = 1;
    [SerializeField] private int m_YawChannel = 2;
    [SerializeField] private int m_ThrottleChannel = 3;
    [SerializeField] private int m_ArmChannel = 7;
    [SerializeField] private int m_ModeChannel = 5;

    [SerializeField] private CrsfMoonController m_CrsfController;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        m_CrsfController.Connect(m_CRSFport, m_CRSFbaud, m_CRSFSendRate);
        m_CrsfController.TelemetryDataReceived += TelemetryDataReceived;
    }

    private void Update()
    {
        m_CrsfController.SetChannel(m_RollChannel, m_ChannelSliders[0].value);
        m_CrsfController.SetChannel(m_PitchChannel, m_ChannelSliders[1].value);
        m_CrsfController.SetChannel(m_ThrottleChannel, m_ChannelSliders[2].value);
        m_CrsfController.SetChannel(m_YawChannel, m_ChannelSliders[3].value);
        m_CrsfController.SetChannel(m_ArmChannel, m_ChannelSliders[4].value);
        m_CrsfController.SetChannel(m_ModeChannel, m_ChannelSliders[5].value);
    }

    private void TelemetryDataReceived(CrsfTelemetryData data)
    {
        m_TelemetryText.text = data.ToString();
    }
}
