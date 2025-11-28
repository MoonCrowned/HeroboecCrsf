import serial
import threading
import time
import struct
from dataclasses import dataclass, field
from typing import List, Callable, Optional, Dict, Any

# --- Data Classes (Structures) ---

@dataclass
class CrsfAnglesData:
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0

@dataclass
class CrsfGpsData:
    latitude: float = 0.0
    longitude: float = 0.0
    ground_speed: float = 0.0  # km/h
    ground_course: float = 0.0 # degrees
    altitude: float = 0.0      # meters
    satellites: int = 0

@dataclass
class CrsfBatteryData:
    voltage: float = 0.0       # Volts
    current: float = 0.0       # Amps
    capacity_mah: int = 0
    remaining_percent: int = 0

@dataclass
class CrsfVSpeedData:
    vertical_speed: float = 0.0 # m/s

@dataclass
class CrsfFlightModeData:
    mode: str = ""

@dataclass
class CrsfTelemetryData:
    angles: CrsfAnglesData = field(default_factory=CrsfAnglesData)
    gps: CrsfGpsData = field(default_factory=CrsfGpsData)
    battery: CrsfBatteryData = field(default_factory=CrsfBatteryData)
    v_speed: CrsfVSpeedData = field(default_factory=CrsfVSpeedData)
    flight_mode: CrsfFlightModeData = field(default_factory=CrsfFlightModeData)

    def __str__(self):
        return (f"Tlm(Pitch={self.angles.pitch:.1f}, Roll={self.angles.roll:.1f}, "
                f"Bat={self.battery.voltage:.1f}V, Mode={self.flight_mode.mode})")

# --- CRC Calculator ---

class CRC8Calculator:
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
    
    def checksum(self, data: bytearray, start: int = 0, length: Optional[int] = None) -> int:
        if length is None:
            length = len(data) - start
        
        crc = 0
        for i in range(start, start + length):
            crc = self._table[crc ^ data[i]]
        return crc

# --- Main Controller Class ---

class CrsfController:
    # Constants
    CRSF_SYNC_BYTE = 0xC8
    
    # Frame Types
    FRAME_GPS = 0x02
    FRAME_VSPEED = 0x07
    FRAME_BATTERY = 0x08
    FRAME_RC_CHANNELS = 0x16
    FRAME_ATTITUDE = 0x1E
    FRAME_FLIGHT_MODE = 0x21

    # Scaling
    CRSF_MIN = 172
    CRSF_MAX = 1811

    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.is_running: bool = False
        
        # Threads
        self.send_thread: Optional[threading.Thread] = None
        self.receive_thread: Optional[threading.Thread] = None
        
        # Channels (0-15) range -1.0 to 1.0
        self.channels: List[float] = [0.0] * 16
        self._channel_lock = threading.Lock()
        
        # Telemetry Storage
        self.telemetry = CrsfTelemetryData()
        
        # Event Callbacks (Subscribers)
        # Format: {'event_name': [func1, func2]}
        self._callbacks: Dict[str, List[Callable[[CrsfTelemetryData], None]]] = {
            'all': [],        # Called on ANY update
            'angles': [],
            'gps': [],
            'battery': [],
            'vspeed': [],
            'flight_mode': []
        }

        self.crc_calculator = CRC8Calculator(0xD5)
        self.send_rate = 50  # Hz
        
        # Stats
        self.stats_bytes_received = 0
        self.stats_packets_sent = 0

    def connect(self, port: str, baud_rate: int = 420000, send_rate: int = 50):
        """Подключение к порту и запуск потоков."""
        if self.is_running:
            print("CRSF уже подключен.")
            return

        try:
            self.channels = [0.0] * 16
            self.send_rate = send_rate
            
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5,
                write_timeout=0.1
            )
            
            self.is_running = True
            
            # Запуск потоков
            self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self.send_thread.start()
            
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            print(f"CRSF подключен к {port} ({baud_rate} baud)")
            
        except Exception as e:
            print(f"CRSF Ошибка подключения: {e}")
            self.disconnect()

    def disconnect(self):
        """Остановка потоков и закрытие порта."""
        self.is_running = False
        
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
            
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                print(f"Ошибка закрытия порта: {e}")
        
        self.serial_port = None
        print("CRSF отключен")

    def set_channel(self, channel_idx: int, value: float):
        """Установка канала (0-15), значение от -1.0 до 1.0."""
        if 0 <= channel_idx < 16:
            with self._channel_lock:
                # Clamp value
                self.channels[channel_idx] = max(-1.0, min(1.0, value))

    def get_telemetry(self) -> CrsfTelemetryData:
        """Возвращает текущий слепок данных телеметрии."""
        return self.telemetry

    # --- Event System ---

    def subscribe(self, event_type: str, callback: Callable[[CrsfTelemetryData], None]):
        """
        Подписка на события.
        event_type: 'angles', 'gps', 'battery', 'vspeed', 'flight_mode', или 'all'
        """
        if event_type in self._callbacks:
            if callback not in self._callbacks[event_type]:
                self._callbacks[event_type].append(callback)
        else:
            print(f"Неизвестный тип события: {event_type}")

    def _trigger_event(self, event_type: str):
        # Call specific subscribers
        for cb in self._callbacks.get(event_type, []):
            try:
                cb(self.telemetry)
            except Exception as e:
                print(f"Error in callback ({event_type}): {e}")
        
        # Call 'all' subscribers
        for cb in self._callbacks['all']:
            try:
                cb(self.telemetry)
            except Exception:
                pass

    # --- Processing Logic ---

    def _process_frame(self, frame: bytearray, length: int):
        if not self._check_crc(frame, length):
            # print("CRC Error")
            return

        frame_type = frame[0]
        payload = frame[1:length-1] # исключаем тип (уже в frame_type) и CRC (последний байт)
        # В C# коде frame включал Type в index 0. Здесь frame передается начиная с Type.
        # frame[0] = Type, frame[1...] = Payload, frame[len-1] = CRC.
        
        # NOTE: В C# коде payload парсится исходя из frame[1], frame[2]... где frame[0] это Type.
        
        updated_event = None

        try:
            if frame_type == self.FRAME_ATTITUDE:
                # Payload: Pitch(2), Roll(2), Yaw(2) BigEndian
                if len(payload) >= 6:
                    p = int.from_bytes(payload[0:2], 'big', signed=True)
                    r = int.from_bytes(payload[2:4], 'big', signed=True)
                    y = int.from_bytes(payload[4:6], 'big', signed=True)
                    
                    self.telemetry.angles.pitch = float(p) * 360.0 / 65536.0
                    self.telemetry.angles.roll = float(r) * 360.0 / 65536.0
                    self.telemetry.angles.yaw = float(y) * 360.0 / 65536.0
                    updated_event = 'angles'

            elif frame_type == self.FRAME_FLIGHT_MODE:
                # Null terminated string or full length
                mode_str = payload.decode('ascii', errors='ignore').split('\x00')[0]
                self.telemetry.flight_mode.mode = mode_str
                updated_event = 'flight_mode'

            elif frame_type == self.FRAME_GPS:
                # Lat(4), Lon(4), Spd(2), Hdg(2), Alt(2), Sats(1)
                if len(payload) >= 15:
                    lat = int.from_bytes(payload[0:4], 'big', signed=True)
                    lon = int.from_bytes(payload[4:8], 'big', signed=True)
                    spd = int.from_bytes(payload[8:10], 'big', signed=False)
                    hdg = int.from_bytes(payload[10:12], 'big', signed=False)
                    alt = int.from_bytes(payload[12:14], 'big', signed=False) # + offset -1000 in C#
                    sats = payload[14]

                    self.telemetry.gps.latitude = lat / 10000000.0
                    self.telemetry.gps.longitude = lon / 10000000.0
                    self.telemetry.gps.ground_speed = spd / 10.0
                    self.telemetry.gps.ground_course = hdg / 100.0
                    self.telemetry.gps.altitude = float(alt) - 1000.0
                    self.telemetry.gps.satellites = sats
                    updated_event = 'gps'

            elif frame_type == self.FRAME_VSPEED:
                if len(payload) >= 2:
                    vspd = int.from_bytes(payload[0:2], 'big', signed=True)
                    self.telemetry.v_speed.vertical_speed = vspd / 100.0
                    updated_event = 'vspeed'

            elif frame_type == self.FRAME_BATTERY:
                # Volt(2), Curr(2), Cap(3), Rem(1)
                if len(payload) >= 8:
                    volt = int.from_bytes(payload[0:2], 'big', signed=True)
                    curr = int.from_bytes(payload[2:4], 'big', signed=True)
                    cap = int.from_bytes(payload[4:7], 'big', signed=False)
                    rem = payload[7]

                    self.telemetry.battery.voltage = volt / 10.0
                    self.telemetry.battery.current = curr / 10.0
                    self.telemetry.battery.capacity_mah = cap
                    self.telemetry.battery.remaining_percent = rem
                    updated_event = 'battery'

            if updated_event:
                self._trigger_event(updated_event)

        except Exception as e:
            print(f"Ошибка парсинга пакета 0x{frame_type:02X}: {e}")

    def _check_crc(self, frame: bytearray, length: int) -> bool:
        # CRC считается от начала (Type) до (Length-1)
        # frame[length-1] это полученный CRC
        crc_received = frame[length - 1]
        crc_calculated = self.crc_calculator.checksum(frame, 0, length - 1)
        return crc_received == crc_calculated

    # --- Loops ---

    def _send_loop(self):
        packet_buffer = bytearray(64) # переиспользуемый буфер? лучше создавать новый
        
        while self.is_running:
            loop_start = time.time()
            
            try:
                if self.serial_port and self.serial_port.is_open:
                    packet = self._create_channels_packet()
                    self.serial_port.write(packet)
                    self.stats_packets_sent += 1
            except Exception as e:
                print(f"Ошибка отправки: {e}")
            
            # Control rate
            elapsed = time.time() - loop_start
            delay = (1.0 / self.send_rate) - elapsed
            if delay > 0:
                time.sleep(delay)

    def _create_channels_packet(self) -> bytes:
        # 1. Подготовка 11-битных значений
        ch_vals = [0] * 16
        with self._channel_lock:
            for i in range(16):
                x = self.channels[i]
                # Map -1..1 to CRSF_MIN..CRSF_MAX
                t = (x + 1.0) * 0.5
                val = int(self.CRSF_MIN + t * (self.CRSF_MAX - self.CRSF_MIN))
                val = max(self.CRSF_MIN, min(self.CRSF_MAX, val))
                ch_vals[i] = val & 0x7FF

        # 2. Упаковка битов (LSB First)
        payload = bytearray(22)
        bit_pos = 0
        for ch in range(16):
            val = ch_vals[ch]
            for b in range(11):
                byte_idx = bit_pos >> 3
                bit_idx = bit_pos & 7
                bit = (val >> b) & 1
                if bit:
                    payload[byte_idx] |= (1 << bit_idx)
                bit_pos += 1
        
        # 3. Сборка пакета [Sync][Len][Type][Payload...][CRC]
        # Length = Type(1) + Payload(22) + CRC(1) = 24
        length = 24
        packet = bytearray()
        packet.append(self.CRSF_SYNC_BYTE)
        packet.append(length)
        packet.append(self.FRAME_RC_CHANNELS)
        packet.extend(payload)
        
        # CRC считается по [Type] + [Payload]
        # Срез packet[2:] берет Type и всё что добавили дальше
        crc_val = self.crc_calculator.checksum(packet, 2)
        packet.append(crc_val)
        
        return bytes(packet)

    def _receive_loop(self):
        """ Читаем поток байт, ищем синхронизацию 0xC8 и разбираем пакеты. """
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    # Ждем хотя бы 2 байта (Sync + Len)
                    if self.serial_port.in_waiting >= 2:
                        sync = self.serial_port.read(1)[0]
                        if sync != self.CRSF_SYNC_BYTE:
                            continue
                        
                        length = self.serial_port.read(1)[0]
                        if length <= 0 or length > 64: # Sanity check
                            continue
                        
                        # Ждем остаток пакета
                        # Можно добавить таймаут, чтобы не зависнуть навечно
                        start_wait = time.time()
                        while self.serial_port.in_waiting < length:
                            if time.time() - start_wait > 0.1:
                                break
                            time.sleep(0.001)
                        
                        if self.serial_port.in_waiting < length:
                            continue # Не дождались

                        # Читаем тело [Type... Payload... CRC]
                        frame_body = self.serial_port.read(length)
                        self.stats_bytes_received += (2 + length)
                        
                        # Обрабатываем. frame_body содержит Type в [0] и CRC в конце
                        self._process_frame(bytearray(frame_body), length)
                    else:
                        time.sleep(0.002)
                else:
                    time.sleep(0.1)
            except Exception as e:
                if self.is_running:
                    print(f"Ошибка приема: {e}")
                    time.sleep(1)