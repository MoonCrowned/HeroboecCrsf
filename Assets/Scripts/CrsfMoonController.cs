using System;
using System.IO.Ports;
using System.Text;
using System.Threading;
using UnityEngine;

public class CrsfMoonController : MonoBehaviour
{
    // События
    public event Action<CrsfTelemetryData> TelemetryDataReceived;
    public event Action<CrsfTelemetryData> TelemetryAnglesDataReceived;
    public event Action<CrsfTelemetryData> TelemetryGpsDataReceived;
    public event Action<CrsfTelemetryData> TelemetryFlightModeDataReceived;
    public event Action<CrsfTelemetryData> TelemetryVSpeedDataReceived;
    public event Action<CrsfTelemetryData> TelemetryBatteryDataReceived;

    // Публичные данные телеметрии
    public CrsfTelemetryData CrsfTelemetry { get; private set; } = new CrsfTelemetryData();

    [SerializeField] int m_DataSent = 0;
    [SerializeField] int m_DataReceived = 0;
    [SerializeField] int m_BytesReceived = 0;

    private SerialPort serialPort;
    private Thread sendThread;
    private Thread receiveThread;
    private bool isRunning = false;

    [SerializeField] private float[] channels = new float[16]; // 16 каналов
    private readonly object channelLock = new object();

    [SerializeField] ushort[] chVals = new ushort[16];

    private int sendRate = 50; // Частота отправки в Гц

    // Константы CRSF
    private const byte CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
    private const byte CRSF_ADDR_MODULE = 0xC8; // TX -> RX (телеметрия от приемника/фк)

    // CRSF channel scaling: -1..1 → 172..1811 (как в OpenTX/Betaflight для 1000–2000us)
    private const int CrsfMin = 172;
    private const int CrsfMax = 1811;

    private bool mTelemetryUpdatedFlag = false;
    private bool mAnglesUpdatedFlag = false;
    private bool mGpsUpdatedFlag = false;
    private bool mFlightModeUpdatedFlag = false;
    private bool mVSpeedUpdatedFlag = false;
    private bool mBatteryUpdatedFlag = false;

    private CRC8Calc mCrc8Calculator = new CRC8Calc(0xD5);
    private long mLastReceivedTelemetryTime = 0;
    [SerializeField] private float mReceiveTelemetryFps = 0f;

    /// <summary>
    /// Подключение к CRSF устройству
    /// </summary>
    public void Connect(string port, int baudRate, int sendChannelsRate)
    {
        if (isRunning)
        {
            Debug.LogWarning("CRSF уже подключен.");
            return;
        }

        try
        {
            //Debug.Log("Connecting... 1");

            // Инициализация каналов в нейтральное положение
            for (int i = 0; i < channels.Length; i++)
            {
                channels[i] = 0f; // -1..1, где 0 - центр
            }

            //Debug.Log("Connecting... 2");

            // Открытие порта
            serialPort = new SerialPort(port, baudRate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 500,
                WriteTimeout = 100,
                Handshake = Handshake.None,
                DtrEnable = false,
                RtsEnable = false,
                NewLine = "\n"
            };

            //Debug.Log("Connecting... 3");

            serialPort.Open();

            //Debug.Log("Connecting... 4");

            sendRate = sendChannelsRate;
            isRunning = true;

            // Запуск потока отправки
            sendThread = new Thread(SendLoop)
            {
                IsBackground = true
            };
            sendThread.Start();

            //Debug.Log("Connecting... 5");

            // Запуск потока приема
            receiveThread = new Thread(ReceiveLoop)
            {
                IsBackground = true
            };
            receiveThread.Start();

            //Debug.Log("Connecting... 6");

            Debug.Log($"CRSF подключен к {port} на скорости {baudRate} baud");
        }
        catch (Exception e)
        {
            Debug.LogError($"Ошибка подключения CRSF: {e.Message}");
            Disconnect();
        }
    }

    /// <summary>
    /// Отключение от CRSF устройства
    /// </summary>
    public void Disconnect()
    {
        isRunning = false;

        // Ожидание завершения потоков
        if (sendThread != null && sendThread.IsAlive)
        {
            sendThread.Join(1000);
        }

        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join(1000);
        }

        // Закрытие порта
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                serialPort.Close();
                serialPort.Dispose();
            }
            catch (Exception e)
            {
                Debug.LogError($"Ошибка закрытия порта: {e.Message}");
            }
        }

        serialPort = null;
        Debug.Log("CRSF отключен");
    }

    /// <summary>
    /// Установить значение канала
    /// </summary>
    /// <param name="channelNum">Номер канала (0-16)</param>
    /// <param name="value">Значение от -1 до 1</param>
    public void SetChannel(int channelNum, float value)
    {
        if (channelNum < 0 || channelNum >= channels.Length)
        {
            Debug.LogError($"Неверный номер канала: {channelNum}");
            return;
        }

        lock (channelLock)
        {
            channels[channelNum] = Mathf.Clamp(value, -1f, 1f);
        }
    }

    /// <summary>
    /// Получить значение канала
    /// </summary>
    /// <param name="channelNum">Номер канала (0-16)</param>
    /// <returns>Значение от -1 до 1</returns>
    public float GetChannel(int channelNum)
    {
        if (channelNum < 0 || channelNum >= channels.Length)
        {
            Debug.LogError($"Неверный номер канала: {channelNum}");
            return 0f;
        }

        lock (channelLock)
        {
            return channels[channelNum];
        }
    }

    private void OnDestroy()
    {
        Disconnect();
    }

    // Поток отправки каналов
    private void SendLoop()
    {
        int delayMs = 1000 / sendRate;

        while (isRunning)
        {
            try
            {
                if (serialPort != null && serialPort.IsOpen)
                {
                    SendCRSFChannels();
                }

                Thread.Sleep(delayMs);
            }
            catch (ThreadAbortException)
            {
                break;
            }
            catch (Exception e)
            {
                Debug.LogError($"Ошибка отправки CRSF: {e.Message}");
                Thread.Sleep(delayMs);
            }
        }
    }

    private byte[] tempBuffer = new byte[255];
    private int tempBufferIndex = 0;

    // Поток приема телеметрии
    private void ReceiveLoop()
    {
        // FIX: буфер побольше и чтение state-machine
        byte[] frame = new byte[255];

        while (isRunning)
        {
            try
            {
                if (serialPort != null && serialPort.IsOpen)
                {
                    // Ждем, когда будут кандидаты на sync и len
                    if (serialPort.BytesToRead >= 2)
                    {
                        int syncByte = serialPort.ReadByte();
                        m_BytesReceived++;

                        if (syncByte < 0)
                            continue;

                        if ((byte)syncByte == 0xC8)
                        {
                            int length = serialPort.ReadByte();
                            m_BytesReceived++;
                            if (length <= 0)
                                continue;

                            // Ждем, когда будет доступно len байт
                            while (serialPort.BytesToRead < length)
                                Thread.Sleep(1);

                            for (int i = 0; i < length; i++)
                                frame[i] = (byte)serialPort.ReadByte();

                            m_BytesReceived += length;

                            // Дебаг принятой строки
                            StringBuilder debugString = new StringBuilder();
                            debugString.Append(length).Append(" ");
                            for (int i = 0; i < length; i++)
                                debugString.Append(frame[i].ToString("X2")).Append(" ");
                            //Debug.Log(debugString.ToString());

                            // Расшифровка
                            ProcessReceivedFrame(frame, length);
                        }
                        else
                            continue;
                    }
                }
                else
                {
                    Thread.Sleep(5);
                }
            }
            catch (TimeoutException)
            {
                // ок
            }
            catch (ThreadAbortException)
            {
                break;
            }
            catch (Exception e)
            {
                Debug.LogError($"Ошибка приема CRSF: {e.Message}");
            }
        }
    }

    private void ProcessReceivedFrame(byte[] frame, int length)
    {
        var crcResult = CheckFrameCrc8(frame, length);
        if (!crcResult)
        {
            Debug.Log("Wrong CRC8");
            return;
        }

        //Debug.Log("Type = " + frame[0].ToString("X2") + " len=" + length);

        // Углы пришли
        if (frame[0] == 0x1E)
        {
            if (length >= 8)
            {
                int a1 = (int)frame[1] * 256 + frame[2];
                int a2 = (int)frame[3] * 256 + frame[4];
                int a3 = (int)frame[5] * 256 + frame[6];

                CrsfTelemetry.Angles.Pitch = (float)(360.0 * (double)a1 / 65536.0);
                CrsfTelemetry.Angles.Yaw = (float)(360.0 * (double)a3 / 65536.0);
                CrsfTelemetry.Angles.Roll = (float)(360.0 * (double)a2 / 65536.0);
                mTelemetryUpdatedFlag = true;
                mAnglesUpdatedFlag = true;

                var time = DateTime.Now.Ticks;
                var deltaTimeTicks = time - mLastReceivedTelemetryTime;
                var deltaTime = (float)(deltaTimeTicks / 10000000.0);
                if (deltaTime < 1f && deltaTime > 0f)
                    mReceiveTelemetryFps = Mathf.Lerp(mReceiveTelemetryFps, 1f / deltaTime, 0.1f);
                mLastReceivedTelemetryTime = time;
            }
        }
        // Flight Mode
        else if (frame[0] == 0x21)
        {
            int payloadLen = length - 2;
            if (payloadLen > 0)
            {
                int end = 1 + payloadLen;
                int strLen = payloadLen;
                for (int i = 1; i < end; i++)
                {
                    if (frame[i] == 0x00)
                    {
                        strLen = i - 1;
                        break;
                    }
                }
                string mode = strLen > 0 ? Encoding.ASCII.GetString(frame, 1, strLen) : string.Empty;
                CrsfTelemetry.FlightMode.Mode = mode;
                mTelemetryUpdatedFlag = true;
                mFlightModeUpdatedFlag = true;

                var time = DateTime.Now.Ticks;
                var deltaTimeTicks = time - mLastReceivedTelemetryTime;
                var deltaTime = (float)(deltaTimeTicks / 10000000.0);
                if (deltaTime < 1f && deltaTime > 0f)
                    mReceiveTelemetryFps = Mathf.Lerp(mReceiveTelemetryFps, 1f / deltaTime, 0.1f);
                mLastReceivedTelemetryTime = time;
            }
        }
        // GPS Position
        else if (frame[0] == 0x02)
        {
            if (length >= 17)
            {
                int latRaw = (frame[1] << 24) | (frame[2] << 16) | (frame[3] << 8) | frame[4];
                int lonRaw = (frame[5] << 24) | (frame[6] << 16) | (frame[7] << 8) | frame[8];
                int spdRaw = (frame[9] << 8) | frame[10];
                int crsRaw = (frame[11] << 8) | frame[12];
                int altRaw = (frame[13] << 8) | frame[14];
                int sats = frame[15];

                CrsfTelemetry.Gps.LatitudeDeg = latRaw / 10000000.0f;
                CrsfTelemetry.Gps.LongitudeDeg = lonRaw / 10000000.0f;
                CrsfTelemetry.Gps.GroundSpeedKmh = spdRaw / 10.0f;
                CrsfTelemetry.Gps.GroundCourseDeg = crsRaw / 100.0f;
                CrsfTelemetry.Gps.AltitudeMeters = (altRaw - 1000);
                CrsfTelemetry.Gps.Satellites = sats;
                mTelemetryUpdatedFlag = true;
                mGpsUpdatedFlag = true;

                var time = DateTime.Now.Ticks;
                var deltaTimeTicks = time - mLastReceivedTelemetryTime;
                var deltaTime = (float)(deltaTimeTicks / 10000000.0);
                if (deltaTime < 1f && deltaTime > 0f)
                    mReceiveTelemetryFps = Mathf.Lerp(mReceiveTelemetryFps, 1f / deltaTime, 0.1f);
                mLastReceivedTelemetryTime = time;
            }
        }
        // Vertical Speed
        else if (frame[0] == 0x07)
        {
            if (length >= 4)
            {
                int vsRaw = (short)((frame[1] << 8) | frame[2]);
                CrsfTelemetry.VSpeed.VerticalSpeedMps = vsRaw / 100.0f;
                mTelemetryUpdatedFlag = true;
                mVSpeedUpdatedFlag = true;

                var time = DateTime.Now.Ticks;
                var deltaTimeTicks = time - mLastReceivedTelemetryTime;
                var deltaTime = (float)(deltaTimeTicks / 10000000.0);
                if (deltaTime < 1f && deltaTime > 0f)
                    mReceiveTelemetryFps = Mathf.Lerp(mReceiveTelemetryFps, 1f / deltaTime, 0.1f);
                mLastReceivedTelemetryTime = time;
            }
        }
        // Battery Voltage and Current
        else if (frame[0] == 0x08)
        {
            if (length >= 10)
            {
                int voltRaw = (short)((frame[1] << 8) | frame[2]);
                int currRaw = (short)((frame[3] << 8) | frame[4]);
                int capRaw = (frame[5] << 16) | (frame[6] << 8) | frame[7];
                int percent = frame[8];

                CrsfTelemetry.Battery.VoltageV = voltRaw / 10.0f;
                CrsfTelemetry.Battery.CurrentA = currRaw / 10.0f;
                CrsfTelemetry.Battery.CapacityMah = capRaw;
                CrsfTelemetry.Battery.RemainingPercent = percent;
                mTelemetryUpdatedFlag = true;
                mBatteryUpdatedFlag = true;

                var time = DateTime.Now.Ticks;
                var deltaTimeTicks = time - mLastReceivedTelemetryTime;
                var deltaTime = (float)(deltaTimeTicks / 10000000.0);
                if (deltaTime < 1f && deltaTime > 0f)
                    mReceiveTelemetryFps = Mathf.Lerp(mReceiveTelemetryFps, 1f / deltaTime, 0.1f);
                mLastReceivedTelemetryTime = time;
            }
        }
    }

    private bool CheckFrameCrc8(byte[] frame, int length)
    {
        var crcReceived = frame[length - 1];
        var crcResult = mCrc8Calculator.Checksum(frame, 0, length - 1);
        //Debug.Log("Crc8Received=" + crcReceived.ToString("X2") + " Crc8Calculated=" + crcResult.ToString("X2"));
        return crcResult == crcReceived;
    }

    // Отправка каналов в формате CRSF
    private void SendCRSFChannels()
    {
        var sendPacket = CreateCrsfChannelsPacket(channels);

        // Отправляем пакет
        serialPort.Write(sendPacket, 0, sendPacket.Length);

        m_DataSent++;
    }

    public byte[] CreateCrsfChannelsPacket(float[] channels)
    {
        if (channels == null) throw new ArgumentNullException(nameof(channels));
        if (channels.Length < 16)
            throw new ArgumentException($"Ожидается 16 каналов.", nameof(channels));

        // 1) Нормализация и квантование в 11-битные значения
        for (int i = 0; i < 16; i++)
        {
            float x = channels[i];
            if (float.IsNaN(x) || float.IsInfinity(x))
                x = 0f;

            // Клипуем в [-1..1]
            if (x < -1f) x = -1f;
            if (x > 1f) x = 1f;

            // Линейное преобразование [-1..1] → [CrsfMin..CrsfMax]
            // x=-1 → 172, x=0 → ~992, x=1 → 1811
            float t = (x + 1f) * 0.5f; // 0..1
            int v = (int)Math.Round(CrsfMin + t * (CrsfMax - CrsfMin));

            if (v < CrsfMin) v = CrsfMin;
            if (v > CrsfMax) v = CrsfMax;

            chVals[i] = (ushort)v; // 11-бит значимое
        }

        // 2) Пакуем 16*11 бит в 22 байта (LSB-first bitstream)
        var payload = new byte[22];
        int bitPos = 0; // позиция в битовом потоке
        for (int ch = 0; ch < 16; ch++)
        {
            int val = chVals[ch] & 0x7FF; // 11 бит
            for (int b = 0; b < 11; b++)
            {
                int byteIndex = bitPos >> 3;
                int bitIndex = bitPos & 7;
                int bit = (val >> b) & 1; // LSB-first
                if (bit != 0)
                    payload[byteIndex] |= (byte)(1 << bitIndex);
                bitPos++;
            }
        }

        // 3) Собираем полный пакет: [Addr][Len][Type][Payload...][CRC]
        // Length = bytes after Length: Type(1) + Payload(22) + CRC(1) = 24
        byte length = (byte)(1 + 22 + 1);

        var packet = new byte[2 + length]; // addr + len + rest
        int idx = 0;
        packet[idx++] = CRSF_ADDR_MODULE;
        packet[idx++] = length;
        packet[idx++] = CRSF_FRAMETYPE_RC_CHANNELS;

        Array.Copy(payload, 0, packet, idx, 22);
        idx += 22;

        // 4) CRC считается по [Type][Payload]
        byte crc = mCrc8Calculator.Checksum(packet, 2, 1 + 22);
        packet[idx++] = crc;

        return packet;
    }

    private void Update()
    {
        if (mTelemetryUpdatedFlag)
        {
            TelemetryDataReceived?.Invoke(CrsfTelemetry);
            mTelemetryUpdatedFlag = false;
        }
        if (mAnglesUpdatedFlag)
        {
            CrsfTelemetry.Angles.lastPacketReceivedTime = Time.time;
            TelemetryAnglesDataReceived?.Invoke(CrsfTelemetry);
            mAnglesUpdatedFlag = false;
        }
        if (mGpsUpdatedFlag)
        {
            CrsfTelemetry.Gps.lastPacketReceivedTime = Time.time;
            TelemetryGpsDataReceived?.Invoke(CrsfTelemetry);
            mGpsUpdatedFlag = false;
        }
        if (mFlightModeUpdatedFlag)
        {
            CrsfTelemetry.FlightMode.lastPacketReceivedTime = Time.time;
            TelemetryFlightModeDataReceived?.Invoke(CrsfTelemetry);
            mFlightModeUpdatedFlag = false;
        }
        if (mVSpeedUpdatedFlag)
        {
            CrsfTelemetry.VSpeed.lastPacketReceivedTime = Time.time;
            TelemetryVSpeedDataReceived?.Invoke(CrsfTelemetry);
            mVSpeedUpdatedFlag = false;
        }
        if (mBatteryUpdatedFlag)
        {
            CrsfTelemetry.Battery.lastPacketReceivedTime = Time.time;
            TelemetryBatteryDataReceived?.Invoke(CrsfTelemetry);
            mBatteryUpdatedFlag = false;
        }
    }
}

/// <summary>
/// Класс данных телеметрии
/// </summary>
public class CrsfTelemetryData
{
    public CrsfFcAnglesData Angles { get; set; } = new CrsfFcAnglesData();
    public CrsfFcGpsData Gps { get; set; } = new CrsfFcGpsData();
    public CrsfFcFlightModeData FlightMode { get; set; } = new CrsfFcFlightModeData();
    public CrsfFcVSpeedData VSpeed { get; set; } = new CrsfFcVSpeedData();
    public CrsfFcBatteryData Battery { get; set; } = new CrsfFcBatteryData();

    // public float Pitch { get { return Angles != null ? Angles.Pitch : 0f; } set { if (Angles == null) Angles = new CrsfFcAnglesData(); Angles.Pitch = value; } }
    // public float Roll { get { return Angles != null ? Angles.Roll : 0f; } set { if (Angles == null) Angles = new CrsfFcAnglesData(); Angles.Roll = value; } }
    // public float Yaw { get { return Angles != null ? Angles.Yaw : 0f; } set { if (Angles == null) Angles = new CrsfFcAnglesData(); Angles.Yaw = value; } }

    public override string ToString()
    {
        //return $"Pitch={Pitch:F1}° Yaw={Yaw:F1}° Roll={Roll:F1}°";
        return $"{Angles}\n{Gps}\n{FlightMode}\n{VSpeed}\n{Battery}";
    }
}

public class CrsfFcAnglesData
{
    public float lastPacketReceivedTime;
    public float Pitch;
    public float Roll;
    public float Yaw;

    public override string ToString()
    {
        return $"Angles = (Pitch={Pitch:F1}° Yaw={Yaw:F1}° Roll={Roll:F1}°)";
    }
}

public class CrsfFcGpsData
{
    public float lastPacketReceivedTime;
    public float LatitudeDeg;
    public float LongitudeDeg;
    public float GroundSpeedKmh;
    public float GroundCourseDeg;
    public float AltitudeMeters;
    public int Satellites;
    public override string ToString()
    {
        return $"GPS = (Lat={LatitudeDeg:F6}° Lon={LongitudeDeg:F6}° Speed={GroundSpeedKmh:F1}km/h Course={GroundCourseDeg:F1}° Alt={AltitudeMeters:F0}m Sats={Satellites})";
    }
}

public class CrsfFcFlightModeData
{
    public float lastPacketReceivedTime;
    public string Mode;
    public override string ToString()
    {
        return $"Mode = {Mode}";
    }
}

public class CrsfFcVSpeedData
{
    public float lastPacketReceivedTime;
    public float VerticalSpeedMps;
    public override string ToString()
    {
        return $"VSpeed = {VerticalSpeedMps:F2}m/s";
    }
}

public class CrsfFcBatteryData
{
    public float lastPacketReceivedTime;
    public float VoltageV;
    public float CurrentA;
    public int CapacityMah;
    public int RemainingPercent;
    public override string ToString()
    {
        return $"Battery = (V={VoltageV:F1}V I={CurrentA:F1}A Used={CapacityMah}mAh Rem={RemainingPercent}%)";
    }
}