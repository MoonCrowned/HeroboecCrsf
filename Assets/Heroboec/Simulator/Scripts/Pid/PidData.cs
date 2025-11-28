using System;

[Serializable]
public struct PidData
{
    public float P;
    public float I;
    public float D;

    public int ErrorHistorySize;
}