using System;
using System.Linq;

[Serializable]
public class PidRegulator2 : IRegulator
{
    private float mIntegral;
    private float mLastError;

    private float mCachedIntegral = 0;
    private float mCachedLastError = 0;

    public float PFactor;
    public float IFactor;
    public float DFactor;

    public float TargetValue { get; set; }
    public float ActualValue { get; set; }
    public float Correction { get; set; }

    private float[] mErrorHistory;
    private int mHistoryIndex = 0;

    public PidRegulator2(float proportionalFactor, float integralFactor, float derivativeFactor, int historySize)
    {
        PFactor = proportionalFactor;
        IFactor = integralFactor;
        DFactor = derivativeFactor;

        mErrorHistory = new float[historySize];
    }

    public PidRegulator2(PidData data)
    {
        PFactor = data.P;
        IFactor = data.I;
        DFactor = data.D;

        mErrorHistory = new float[data.ErrorHistorySize];
        mHistoryIndex = 0;
    }

    public void UpdateConfig(PidData data)
    {
        PFactor = data.P;
        IFactor = data.I;
        DFactor = data.D;

        if (mErrorHistory.Length != data.ErrorHistorySize)
        {
            mErrorHistory = new float[data.ErrorHistorySize];
            mHistoryIndex = 0;
        }
    }

    public void UpdateCorrection(float deltaTime)
    {
        var error = TargetValue - ActualValue;
        if (mErrorHistory == null || mErrorHistory.Length == 0)
            mIntegral += error * deltaTime;
        else
        {
            mHistoryIndex++;
            if (mHistoryIndex >= mErrorHistory.Length)
                mHistoryIndex = 0;
            mErrorHistory[mHistoryIndex] = error;
            mIntegral = mErrorHistory.Sum();
        }

        var derivative = (error - mLastError) / deltaTime;
        mLastError = error;

        var correctionValue = error * PFactor + mIntegral * IFactor + derivative * DFactor;

        Correction = correctionValue;
    }

    public void Reset()
    {
        mIntegral = 0f;
        mLastError = 0f;
        Correction = 0f;
        for (int k = 0; k < mErrorHistory.Length; k++)
            mErrorHistory[k] = 0;
        mHistoryIndex = 0;
    }

    public void CacheInternalData()
    {
        mCachedIntegral = mIntegral;
        mCachedLastError = mLastError;
    }

    public void RestoreCacheInternalData()
    {
        mIntegral = mCachedIntegral;
        mLastError = mCachedLastError;
    }
}