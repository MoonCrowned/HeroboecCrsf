public interface IRegulator
{
    public float TargetValue { get; set; }
    public float ActualValue { get; set; }
    public float Correction { get; set; }

    public void UpdateCorrection(float deltaTime);
    public void Reset();
    public void CacheInternalData();
    public void RestoreCacheInternalData();
    public void UpdateConfig(PidData data);
}