using System;
using Cysharp.Threading.Tasks;
using UnityEngine;

public class HeroboecSimulator2 : MonoBehaviour
{
    [SerializeField] private CrsfMoonController m_CrsfMoonController;

    [SerializeField] private float m_ToHalfThrottleTime = 1f;
    [SerializeField] private float m_HalfThrottle = 0f;
    [SerializeField] private int m_HalfToMaxThrottleDelay = 1000;
    [SerializeField] private float m_ToMaxThrottleTime = 1f;
    [SerializeField] private float m_MaxThrottle = 1f;
    [SerializeField] private int m_LiftOffTime = 2000;
    [SerializeField] private Vector2 m_PitchFromTo;
    [SerializeField] private float m_PitchFromToTime = 2f;
    [SerializeField] private int m_FlyTime = 10;
    [SerializeField] private AnimationCurve m_FlyYawCurve;
    [SerializeField] private float m_FlyYawCurveFactor = 2f;

    private bool mRunning = false;


    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            StartStage().Forget();
        }

        if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            StopAll();
        }
    }

    public async UniTask StartStage()
    {
        mRunning = true;
        
        m_CrsfMoonController.SetChannel(0, 0); // Roll
        m_CrsfMoonController.SetChannel(1, 0); // Pitch
        m_CrsfMoonController.SetChannel(2, -1); // Throttle
        m_CrsfMoonController.SetChannel(3, 0); // Yaw
        m_CrsfMoonController.SetChannel(4, 1); // Arm
        m_CrsfMoonController.SetChannel(5, 1); // Mode ACRO
        
        Debug.Log("Init stage");

        await UniTask.Delay(3000);
        if (!mRunning) return;
        
        Debug.Log("Before half throttle");

        await TweenAlpha(m_ToHalfThrottleTime, (a) => m_CrsfMoonController.SetChannel(2, Mathf.Lerp(-1f, m_HalfThrottle, a)));
        if (!mRunning) return;
        
        Debug.Log("After half throttle");

        await UniTask.Delay(m_HalfToMaxThrottleDelay);
        if (!mRunning) return;
        
        Debug.Log("Before max throttle");
        
        await TweenAlpha(m_ToMaxThrottleTime, (a) => m_CrsfMoonController.SetChannel(2, Mathf.Lerp(m_HalfThrottle, m_MaxThrottle, a)));
        if (!mRunning) return;
        
        Debug.Log("Liftoff timer");

        await UniTask.Delay(m_LiftOffTime);
        if (!mRunning) return;
        
        Debug.Log("To Horizon 2 - horizon mode");
        
        m_CrsfMoonController.SetChannel(5, 0); // Mode Horizon
        m_CrsfMoonController.SetChannel(1, m_PitchFromTo.x); // Pitch
        await UniTask.Delay(300);
        if (!mRunning) return;
        
        Debug.Log("To Horizon 3 - max throttle");
        
        m_CrsfMoonController.SetChannel(2, m_MaxThrottle); // Throttle
        await UniTask.Delay(300);
        if (!mRunning) return;
        
        Debug.Log("To Horizon 4 - pitch to horizon");
        
        await TweenAlpha(m_PitchFromToTime,
            (a) => m_CrsfMoonController.SetChannel(1, Mathf.Lerp(m_PitchFromTo.x, m_PitchFromTo.y, a)),
            () => !mRunning);
        if (!mRunning) return;
        
        Debug.Log("Yaw maneures");

        await TweenAlpha(m_FlyTime,
            (a) => m_CrsfMoonController.SetChannel(3, m_FlyYawCurve.Evaluate(a) * m_FlyYawCurveFactor),
            () => !mRunning);
        if (!mRunning) return;
        
        Debug.Log("Stop");

        StopAll().Forget();
    }

    public async UniTask StopAll()
    {
        mRunning = false;
        
        m_CrsfMoonController.SetChannel(0, 0);
        m_CrsfMoonController.SetChannel(1, 0);
        m_CrsfMoonController.SetChannel(2, -1);
        m_CrsfMoonController.SetChannel(3, 0);

        await UniTask.Delay(500);

        m_CrsfMoonController.SetChannel(4, 0);
    }

    public async UniTask TweenAlpha(float time, Action<float> action, Func<bool> cancelCondition = null)
    {
        var startTime = Time.time;
        while (Time.time - startTime < time)
        {
            var alpha = (Time.time - startTime) / time;
            action?.Invoke(alpha);
            await UniTask.Yield();
            
            if( cancelCondition != null)
                if( cancelCondition() )
                    return;
        }

        action?.Invoke(1f);
    }
}