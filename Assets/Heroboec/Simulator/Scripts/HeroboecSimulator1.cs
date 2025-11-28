using System;
using Cysharp.Threading.Tasks;
using UnityEngine;

public class HeroboecSimulator1 : MonoBehaviour
{
    [SerializeField] private CrsfMoonController m_CrsfMoonController;

    [SerializeField] private Transform m_HeroboecTransform;
    [SerializeField] private Vector3 m_HeroboecVelocity;

    [SerializeField] private float m_NullYaw;

    [SerializeField] private float m_Drag;
    [SerializeField] private float m_WingForce;

    [SerializeField] private float m_Stage1_Height;
    [SerializeField] private float m_Stage1_MaxThrottle = 0f;

    [SerializeField] private PidData m_PidPitch;
    [SerializeField] private PidData m_PidRoll;
    [SerializeField] private PidData m_PidYaw;

    [SerializeField, Range(0, 0.2f)] private float mPidP = 0.05f;
    [SerializeField, Range(0, 0.2f)] private float mPidD = 0.05f;

    private bool mPidsTest = false;

    private IRegulator pidPitch;
    private IRegulator pidRoll;
    private IRegulator pidYaw;

    [SerializeField, Range(-180f, 180f)] private float mTargetPitch = 0f;
    [SerializeField, Range(-180f, 180f)] private float mTargetRoll = 0f;
    [SerializeField, Range(-180f, 180f)] private float mTargetYaw = 0f;


    private void Start()
    {
        pidPitch = new PidRegulator2(m_PidPitch);
        pidRoll = new PidRegulator2(m_PidRoll);
        pidYaw = new PidRegulator2(m_PidYaw);
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            StartStage1().Forget();
        }

        if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            StopAll();
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            mPidsTest = !mPidsTest;
            if (mPidsTest)
            {
                m_NullYaw = m_HeroboecTransform.localEulerAngles.y;
            }
        }
        
        m_PidPitch.P = m_PidYaw.P = mPidP;
        m_PidPitch.D = m_PidYaw.D = mPidD;

        if (mPidsTest)
        {
            var targetPitch = mTargetPitch;
            var targetYaw = mTargetYaw;
            var targetRoll = mTargetRoll;
            
            pidPitch.UpdateConfig(m_PidPitch);
            pidRoll.UpdateConfig(m_PidRoll);
            pidYaw.UpdateConfig(m_PidYaw);

            pidPitch.TargetValue = targetPitch;
            pidPitch.ActualValue = Mathf.DeltaAngle(0f, m_HeroboecTransform.localEulerAngles.x);
            //pidPitch.UpdateCorrection(Mathf.Clamp(Time.deltaTime, 0.01f, 0.05f));
            pidPitch.UpdateCorrection(0.02f);
            //Debug.Log("PITCH t="+pidPitch.TargetValue+", a="+pidPitch.ActualValue+" c="+pidPitch.Correction);

            pidYaw.TargetValue = targetYaw;
            pidYaw.ActualValue = Mathf.DeltaAngle(0f, m_HeroboecTransform.localEulerAngles.y - m_NullYaw);
            //pidYaw.UpdateCorrection(Mathf.Clamp(Time.deltaTime, 0.01f, 0.05f));
            pidYaw.UpdateCorrection(0.02f);
            //Debug.Log("t="+pidYaw.TargetValue+" a="+pidYaw.ActualValue+" c="+pidYaw.Correction);
            
            
            pidRoll.TargetValue = targetRoll;
            pidRoll.ActualValue = Mathf.DeltaAngle(0f, m_HeroboecTransform.localEulerAngles.z);
            //pidRoll.UpdateCorrection(Mathf.Clamp(Time.deltaTime, 0.01f, 0.05f));
            pidRoll.UpdateCorrection(0.02f);
            //Debug.Log("t="+pidRoll.TargetValue+" a="+pidRoll.ActualValue+" c="+pidRoll.Correction);

            //Debug.Log(pidPitch.Correction + " " + pidYaw.Correction + " " + pidRoll.Correction);
            
            m_CrsfMoonController.SetChannel(1, pidPitch.Correction);
            m_CrsfMoonController.SetChannel(3, pidYaw.Correction);
            m_CrsfMoonController.SetChannel(0, -pidRoll.Correction);
        }
    }

    public async UniTask StartStage1()
    {
        m_NullYaw = m_HeroboecTransform.localEulerAngles.y;

        m_CrsfMoonController.SetChannel(0, 0); // Roll
        m_CrsfMoonController.SetChannel(1, 0); // Pitch
        m_CrsfMoonController.SetChannel(2, -1); // Throttle
        m_CrsfMoonController.SetChannel(3, 0); // Yaw
        m_CrsfMoonController.SetChannel(4, 1);
        m_CrsfMoonController.SetChannel(5, 1);

        await UniTask.Delay(1000);

        await TweenAlpha(2f, (a) => m_CrsfMoonController.SetChannel(2, Mathf.Lerp(-1f, m_Stage1_MaxThrottle, a)));
    }

    public async UniTask StopAll()
    {
        mPidsTest = false;
        
        m_CrsfMoonController.SetChannel(0, 0);
        m_CrsfMoonController.SetChannel(1, 0);
        m_CrsfMoonController.SetChannel(2, -1);
        m_CrsfMoonController.SetChannel(3, 0);

        await UniTask.Delay(500);

        m_CrsfMoonController.SetChannel(4, 0);
    }

    public async UniTask TweenAlpha(float time, Action<float> action)
    {
        var startTime = Time.time;
        while (Time.time - startTime < time)
        {
            var alpha = (Time.time - startTime) / time;
            action?.Invoke(alpha);
            await UniTask.Yield();
        }

        action?.Invoke(1f);
    }
}