# crsf_controller.py
import serial
import threading
import time
from typing import Callable, Optional
import math

class CrsfTelemetryData:
    """Класс данных телеметрии"""
    def __init__(self):
        self.pitch: float = 0.0  # градусы
        self.roll: float = 0.0   # градусы
        self.yaw: float = 0.0    # градусы
    
    def __str__(self):
        return f"Pitch={self.pitch:.1f}° Yaw={self.yaw:.1f}° Roll={self.roll:.1f}°"


class CRC8Calculator:
    """Калькулятор CRC8 для CRSF"""
    def __init__(self, polynomial: int = 0xD5):
        self.polynomial = polynomial
        self._table = self._generate_table()
    
    def _generate_table(self):
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ self.polynomial) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
            table.append(crc)
        return table
    
    def checksum(self, data: bytes, start: int = 0, length: Optional[int] = None) -> int:
        """Вычисление CRC8"""
        if length is None:
            length = len(data) - start
        
        crc = 0
        for i in range(start, start + length):
            crc = self._table[crc ^ data[i]]
        return crc


class CrsfController:
    """Контроллер для работы с CRSF протоколом"""
    
    # Константы CRSF
    CRSF_FRAMETYPE_RC_CHANNELS = 0x16
    CRSF_FRAMETYPE_ATTITUDE = 0x1E
    CRSF_ADDR_MODULE = 0xC8
    CRSF_MIN = 172
    CRSF_MAX = 1811
    
    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.send_thread: Optional[threading.Thread] = None
        self.receive_thread: Optional[threading.Thread] = None
        self.is_running: bool = False
        
        self.channels = [0.0] * 16  # 16 каналов
        self.channel_lock = threading.Lock()
        
        self.telemetry = CrsfTelemetryData()
        self._telemetry_callbacks = []
        
        self.send_rate: int = 50
        self.crc_calculator = CRC8Calculator(0xD5)
        
        # Статистика
        self.data_sent: int = 0
        self.data_received: int = 0
        self.bytes_received: int = 0
        
        self._last_telemetry_time: float = 0.0
        self.telemetry_fps: float = 0.0
    
    def connect(self, port: str, baud_rate: int, send_rate: int = 50):
        """
        Подключение к CRSF устройству
        
        Args:
            port: COM порт (например, 'COM5' или '/dev/ttyUSB0')
            baud_rate: Скорость (например, 420000)
            send_rate: Частота отправки каналов в Гц
        """
        if self.is_running:
            print("CRSF уже подключен.")
            return
        
        try:
            # Инициализация каналов в нейтральное положение
            for i in range(len(self.channels)):
                self.channels[i] = 0.0
            
            # Открытие порта
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5,
                write_timeout=0.1
            )
            
            self.send_rate = send_rate
            self.is_running = True
            
            # Запуск потока отправки
            self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self.send_thread.start()
            
            # Запуск потока приема
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            print(f"CRSF подключен к {port} на скорости {baud_rate} baud")
        
        except Exception as e:
            print(f"Ошибка подключения CRSF: {e}")
            self.disconnect()
    
    def disconnect(self):
        """Отключение от CRSF устройства"""
        self.is_running = False
        
        # Ожидание завершения потоков
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        
        # Закрытие порта
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                print(f"Ошибка закрытия порта: {e}")
        
        self.serial_port = None
        print("CRSF отключен")
    
    def set_channel(self, channel_num: int, value: float):
        """
        Установить значение канала
        
        Args:
            channel_num: Номер канала (0-15)
            value: Значение от -1 до 1
        """
        if channel_num < 0 or channel_num >= len(self.channels):
            print(f"Неверный номер канала: {channel_num}")
            return
        
        with self.channel_lock:
            self.channels[channel_num] = max(-1.0, min(1.0, value))
    
    def get_channel(self, channel_num: int) -> float:
        """
        Получить значение канала
        
        Args:
            channel_num: Номер канала (0-15)
            
        Returns:
            Значение от -1 до 1
        """
        if channel_num < 0 or channel_num >= len(self.channels):
            print(f"Неверный номер канала: {channel_num}")
            return 0.0
        
        with self.channel_lock:
            return self.channels[channel_num]
    
    def add_telemetry_callback(self, callback: Callable[[CrsfTelemetryData], None]):
        """
        Добавить callback для получения телеметрии
        
        Args:
            callback: Функция, которая будет вызвана при получении телеметрии
        """
        if callback not in self._telemetry_callbacks:
            self._telemetry_callbacks.append(callback)
    
    def remove_telemetry_callback(self, callback: Callable[[CrsfTelemetryData], None]):
        """Удалить callback для телеметрии"""
        if callback in self._telemetry_callbacks:
            self._telemetry_callbacks.remove(callback)
    
    def _send_loop(self):
        """Поток отправки каналов"""
        delay_sec = 1.0 / self.send_rate
        
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    self._send_crsf_channels()
                
                time.sleep(delay_sec)
            
            except Exception as e:
                print(f"Ошибка отправки CRSF: {e}")
                time.sleep(delay_sec)
    
    def _receive_loop(self):
        """Поток приема телеметрии"""
        frame = bytearray(255)
        
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    # Ждем sync byte
                    if self.serial_port.in_waiting >= 2:
                        sync_byte = self.serial_port.read(1)[0]
                        self.bytes_received += 1
                        
                        if sync_byte == 0xC8:
                            length = self.serial_port.read(1)[0]
                            self.bytes_received += 1
                            
                            if length > 0:
                                # Ждем полного пакета
                                while self.serial_port.in_waiting < length:
                                    time.sleep(0.001)
                                
                                frame_data = self.serial_port.read(length)
                                self.bytes_received += length
                                
                                # Обработка пакета
                                self._process_received_frame(frame_data, length)
                else:
                    time.sleep(0.005)
            
            except serial.SerialTimeoutException:
                pass
            except Exception as e:
                print(f"Ошибка приема CRSF: {e}")
    
    def _process_received_frame(self, frame: bytes, length: int):
        """Обработка полученного пакета"""
        # Проверка CRC
        if not self._check_frame_crc8(frame, length):
            print("Wrong CRC8")
            return
        
        frame_type = frame[0]
        
        # Телеметрия углов (Attitude)
        if frame_type == self.CRSF_FRAMETYPE_ATTITUDE:
            if length >= 7:
                a1 = (frame[1] << 8) | frame[2]
                a2 = (frame[3] << 8) | frame[4]
                a3 = (frame[5] << 8) | frame[6]
                
                self.telemetry.pitch = (360.0 * a1) / 65536.0
                self.telemetry.yaw = (360.0 * a3) / 65536.0
                self.telemetry.roll = (360.0 * a2) / 65536.0
                
                # Вычисление FPS телеметрии
                current_time = time.time()
                if self._last_telemetry_time > 0:
                    delta_time = current_time - self._last_telemetry_time
                    if 0 < delta_time < 1.0:
                        fps = 1.0 / delta_time
                        self.telemetry_fps = self.telemetry_fps * 0.9 + fps * 0.1
                self._last_telemetry_time = current_time
                
                # Вызов callbacks
                for callback in self._telemetry_callbacks:
                    try:
                        callback(self.telemetry)
                    except Exception as e:
                        print(f"Ошибка в callback телеметрии: {e}")
        
        # Каналы
        elif frame_type == self.CRSF_FRAMETYPE_RC_CHANNELS:
            pass  # Можно добавить обработку при необходимости
    
    def _check_frame_crc8(self, frame: bytes, length: int) -> bool:
        """Проверка CRC8 пакета"""
        crc_received = frame[length - 1]
        crc_calculated = self.crc_calculator.checksum(frame, 0, length - 1)
        return crc_received == crc_calculated
    
    def _send_crsf_channels(self):
        """Отправка каналов в формате CRSF"""
        with self.channel_lock:
            packet = self._create_crsf_channels_packet(self.channels)
        
        self.serial_port.write(packet)
        self.data_sent += 1
    
    def _create_crsf_channels_packet(self, channels: list) -> bytes:
        """Создание пакета с каналами"""
        if len(channels) < 16:
            raise ValueError("Ожидается 16 каналов")
        
        # Квантование каналов в 11-битные значения
        ch_vals = []
        for i in range(16):
            x = channels[i]
            
            # Клипуем в [-1..1]
            x = max(-1.0, min(1.0, x))
            
            # Линейное преобразование [-1..1] → [172..1811]
            t = (x + 1.0) * 0.5  # 0..1
            v = int(round(self.CRSF_MIN + t * (self.CRSF_MAX - self.CRSF_MIN)))
            v = max(self.CRSF_MIN, min(self.CRSF_MAX, v))
            
            ch_vals.append(v & 0x7FF)  # 11 бит
        
        # Пакуем 16*11 бит в 22 байта (LSB-first)
        payload = bytearray(22)
        bit_pos = 0
        
        for ch in range(16):
            val = ch_vals[ch]
            for b in range(11):
                byte_index = bit_pos >> 3
                bit_index = bit_pos & 7
                bit = (val >> b) & 1
                if bit:
                    payload[byte_index] |= (1 << bit_index)
                bit_pos += 1
        
        # Собираем пакет: [Addr][Len][Type][Payload][CRC]
        length = 1 + 22 + 1  # Type + Payload + CRC
        packet = bytearray()
        packet.append(self.CRSF_ADDR_MODULE)
        packet.append(length)
        packet.append(self.CRSF_FRAMETYPE_RC_CHANNELS)
        packet.extend(payload)
        
        # CRC считается по [Type][Payload]
        crc = self.crc_calculator.checksum(packet, 2, 1 + 22)
        packet.append(crc)
        
        return bytes(packet)
    
    def __del__(self):
        """Деструктор"""
        self.disconnect()