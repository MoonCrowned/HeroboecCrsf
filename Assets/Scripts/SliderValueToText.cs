using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class SliderValueToText : MonoBehaviour
{
    [SerializeField] private string m_Format = "0.00";

    [SerializeField] private Slider m_Slider;
    [SerializeField] private TMP_Text m_Text;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        m_Slider.onValueChanged.AddListener((_) =>
        {
            m_Text.text = m_Slider.value.ToString(m_Format);
        });   
        m_Text.text = m_Slider.value.ToString(m_Format);
    }
}
