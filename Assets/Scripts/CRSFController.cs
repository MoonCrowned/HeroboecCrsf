using System;
using System.IO.Ports;
using UnityEngine;

public class CRSFController : MonoBehaviour
{
    // Настройки подключения
    public string portName = "COM3"; // Укажите ваш порт
    public int baudRate = 420000;    // Скорость передачи для CRSF

    private SerialPort serialPort;

    // Данные джойстика
    private float roll, pitch, throttle, yaw;

    void Start()
    {
        // Инициализация последовательного порта
        serialPort = new SerialPort(portName, baudRate);
        try
        {
            serialPort.Open();
            Debug.Log("Подключение к CRSF установлено.");
        }
        catch (Exception e)
        {
            Debug.LogError($"Не удалось подключиться к CRSF: {e.Message}");
        }
    }

    void Update()
    {
        // Считываем данные от джойстика
        roll = Input.GetAxis("Horizontal");       // Ось крена
        pitch = Input.GetAxis("Vertical");        // Ось тангажа
        throttle = Input.GetAxis("Throttle");     // Ось газа
        yaw = Input.GetAxis("Yaw");               // Ось рыскания

        // Преобразуем данные джойстика в каналы CRSF
        int[] channels = new int[16];
        channels[0] = JoystickToCRSFValue(roll);       // Канал 1: Крен
        channels[1] = JoystickToCRSFValue(pitch);      // Канал 2: Тангаж
        channels[2] = JoystickToCRSFValue(throttle);   // Канал 3: Газ
        channels[3] = JoystickToCRSFValue(yaw);        // Канал 4: Рыскание

        // Отправляем каналы в CRSF
        SendCRSFChannels(channels);
    }

    void OnDestroy()
    {
        // Закрываем порт при завершении работы
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
        }
    }

    // Преобразует значение джойстика (-1..1) в 11-битное значение CRSF (172..1811)
    int JoystickToCRSFValue(float value)
    {
        return Mathf.Clamp((int)((value + 1) * (1811 - 172) / 2 + 172), 172, 1811);
    }

    // Отправляет каналы управления в формате CRSF
    void SendCRSFChannels(int[] channels)
    {
        if (serialPort == null || !serialPort.IsOpen)
        {
            Debug.LogError("Последовательный порт не открыт.");
            return;
        }

        // Создаем пакет CRSF
        byte[] packet = new byte[23];
        packet[0] = 0x0F; // Заголовок (тип пакета: каналы управления)
        packet[1] = 0x16; // Длина пакета (22 байта данных)

        // Заполняем каналы управления
        for (int i = 0; i < 16; i++)
        {
            int channelValue = channels[i];
            packet[2 + i * 2] = (byte)(channelValue & 0xFF);         // Младший байт
            packet[3 + i * 2] = (byte)((channelValue >> 8) & 0x07);  // Старший байт (только 3 бита)
        }

        // Вычисляем CRC
        byte crc = CalculateCRC(packet, 22);
        packet[22] = crc; // Добавляем CRC в конец пакета

        // Отправляем пакет
        serialPort.Write(packet, 0, 23);
    }

    // Вычисляет CRC для пакета CRSF
    byte CalculateCRC(byte[] data, int length)
    {
        byte crc = 0;
        for (int i = 0; i < length; i++)
        {
            crc ^= data[i];
        }
        return crc;
    }
}