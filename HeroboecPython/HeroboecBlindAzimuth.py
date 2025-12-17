#!/usr/bin/env python3

# HeroboecBlindAzimuth.py
import time
import json
import os
import threading
import sys
import select
from typing import Optional
from CRSF import CrsfController, CrsfTelemetryData

try:
    import gpiod
except ImportError:
    gpiod = None


_log_file = None


def init_logger():
    global _log_file
    if _log_file is not None:
        return
    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(base_dir, f"{ts}_log.txt")
        _log_file = open(log_path, "w", encoding="utf-8")
    except Exception:
        _log_file = None


def log_print(*args, **kwargs):
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\n")
    text = sep.join(str(a) for a in args)

    # Консоль
    sys.stdout.write(text + end)
    sys.stdout.flush()

    # Лог-файл
    if _log_file is not None:
        try:
            _log_file.write(text + end)
            _log_file.flush()
        except Exception:
            pass


class HeroboecBlindAzimuthScenario:
    """Сценарий полета с поддержанием азимута и высоты по телеметрии"""

    def __init__(self, controller: CrsfController, config_path: Optional[str] = None):
        self.controller = controller
        self.running = False

        if config_path is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(base_dir, "HeroboecBlindAzimuth_config.json")

        self._load_config(config_path)

        self.zero_altitude = 0.0
        self.target_azimuth = 0.0
        self.target_height = 0.0  # относительная высота от нулевой точки

        self.azimuth_loop_running = False
        self.height_loop_running = False
        self.azimuth_thread: Optional[threading.Thread] = None
        self.height_thread: Optional[threading.Thread] = None

        self._attitude_lock = threading.Lock()
        self.base_pitch_angle = 0.0
        self.turn_pitch_offset = 0.0
        self.desired_roll = 0.0
        self.desired_yaw = 0.0

    def _load_config(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        self.port = str(cfg.get("port", "COM5"))

        self.to_half_throttle_time = float(cfg.get("to_half_throttle_time", 1.0))
        self.half_throttle = float(cfg.get("half_throttle", 0.0))
        self.half_to_max_throttle_delay = float(cfg.get("half_to_max_throttle_delay", 1.0))
        self.to_max_throttle_time = float(cfg.get("to_max_throttle_time", 1.0))
        self.max_throttle = float(cfg.get("max_throttle", 1.0))
        self.liftoff_time = float(cfg.get("liftoff_time", 4.0))
        self.pitch_from = float(cfg.get("pitch_from", -0.5))
        self.to_angle_delay = float(cfg.get("to_angle_delay", 2.0))

        self.azimuth_kp = float(cfg.get("azimuth_kp", 0.02))
        self.yaw_control_min = float(cfg.get("yaw_control_min", -0.5))
        self.yaw_control_max = float(cfg.get("yaw_control_max", 0.5))
        self.roll_control_min = float(cfg.get("roll_control_min", -0.4))
        self.roll_control_max = float(cfg.get("roll_control_max", 0.4))
        self.pitch_turn_min = float(cfg.get("pitch_turn_min", -0.2))
        self.pitch_turn_max = float(cfg.get("pitch_turn_max", 0.2))

        self.height_kp = float(cfg.get("height_kp", 0.1))
        self.pitch_height_min = float(cfg.get("pitch_height_min", -0.4))
        self.pitch_height_max = float(cfg.get("pitch_height_max", 0.4))

        self.change_azimuth_speed = float(cfg.get("change_azimuth_speed", 10.0))

        self.trigger_hold_time = float(cfg.get("trigger_hold_time", 2.0))

        self.program_azimuth1 = float(cfg.get("program_azimuth1", 0.0))
        self.program_height1 = float(cfg.get("program_height1", 20.0))
        self.program_1_2_pause = float(cfg.get("program_1_2_pause", 10.0))

        self.program_azimuth2 = float(cfg.get("program_azimuth2", 90.0))
        self.program_height2 = float(cfg.get("program_height2", 25.0))
        self.program_2_3_pause = float(cfg.get("program_2_3_pause", 10.0))

        self.program_azimuth3 = float(cfg.get("program_azimuth3", 180.0))
        self.program_height3 = float(cfg.get("program_height3", 20.0))
        self.program_3_4_pause = float(cfg.get("program_3_4_pause", 10.0))

        self.min_sats_for_gps = int(cfg.get("min_sats_for_gps", 4))

    def stop_all(self):
        log_print("Остановка сценария...")
        self.running = False
        self.azimuth_loop_running = False
        self.height_loop_running = False

        if self.azimuth_thread and self.azimuth_thread.is_alive():
            self.azimuth_thread.join(timeout=1.0)
        if self.height_thread and self.height_thread.is_alive():
            self.height_thread.join(timeout=1.0)

        time.sleep(0.5)

        self.controller.set_channel(0, 0)    # Roll
        self.controller.set_channel(1, 0)    # Pitch
        self.controller.set_channel(2, -1)   # Throttle
        self.controller.set_channel(3, 0)    # Yaw
        time.sleep(0.5)
        self.controller.set_channel(4, 0)    # Disarm
        time.sleep(0.5)
        log_print("Остановлено")

    def tween_alpha(self, duration: float, callback, check_running: bool = True) -> bool:
        start_time = time.time()
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                callback(1.0)
                break

            alpha = elapsed / duration
            callback(alpha)

            if check_running and not self.running:
                return False

            time.sleep(0.02)
        return True

    def _clamp(self, v: float, vmin: float, vmax: float) -> float:
        if v < vmin:
            return vmin
        if v > vmax:
            return vmax
        return v
        
    def lerp(self, a: float, b: float, t: float) -> float:
        return self._clamp(a + (b - a) * t, a, b)

    def _apply_attitude(self):
        with self._attitude_lock:
            roll = self.desired_roll
            pitch = self.base_pitch_angle + self.turn_pitch_offset
            yaw = self.desired_yaw
        self.controller.set_channel(0, roll)
        self.controller.set_channel(1, pitch)
        self.controller.set_channel(3, yaw)

    def _angle_error(self, current_deg: float, target_deg: float) -> float:
        # Ошибка в диапазоне [-180, 180]
        diff = (target_deg - current_deg + 900.0) % 360.0 - 180.0
        return diff

    def _normalize_angle(self, deg: float) -> float:
        return (deg + 900.0) % 360.0 - 180.0

    def _tween_target_azimuth(self, target_deg: float) -> bool:
        tlm = self.controller.get_telemetry()
        current = self._normalize_angle(tlm.angles.yaw - self.zero_azimuth)
        target_norm = self._normalize_angle(target_deg)
        diff = self._angle_error(current, target_norm)
        distance = abs(diff)
        if distance < 1e-3 or self.change_azimuth_speed <= 0.0:
            self.target_azimuth = target_norm
            return True

        duration = distance / self.change_azimuth_speed
        start = current

        def _update(alpha: float):
            angle = self._normalize_angle(start + diff * alpha)
            self.target_azimuth = angle

        return self.tween_alpha(duration, _update)

    def _start_control_loops(self):
        if self.azimuth_loop_running or self.height_loop_running:
            return
        self.azimuth_loop_running = True
        self.height_loop_running = True
        self.azimuth_thread = threading.Thread(target=self._azimuth_loop, daemon=True)
        self.height_thread = threading.Thread(target=self._height_loop, daemon=True)
        self.azimuth_thread.start()
        self.height_thread.start()

    def _azimuth_loop(self):
        # Контур поддержания азимута. Использует yaw, roll и небольшой pitch-offset.
        while self.running and self.azimuth_loop_running:
            tlm = self.controller.get_telemetry()

            # if tlm.gps.satellites < self.min_sats_for_gps:
            #     time.sleep(0.2)
            #     continue

            # current_azimuth = tlm.gps.ground_course
            current_azimuth = tlm.angles.yaw - self.zero_azimuth
            error = self._angle_error(current_azimuth, self.target_azimuth)

            control = self._clamp(error * self.azimuth_kp, -1.0, 1.0)

            t = (control + 1.0) * 0.5
            yaw = self.lerp(self.yaw_control_min, self.yaw_control_max, t)
            roll = self.lerp(self.roll_control_min, self.roll_control_max, (control*3 + 1.0) * 0.5)
            pitch_turn = self.lerp(self.pitch_turn_min, self.pitch_turn_max, t)

            with self._attitude_lock:
                self.desired_yaw = yaw
                self.desired_roll = roll
                self.turn_pitch_offset = pitch_turn

            log_print(f"_A ca={current_azimuth:.1f} ta={self.target_azimuth:.1f} y={tlm.angles.yaw:.1f} r={tlm.angles.roll:.1f} p={tlm.angles.pitch:.1f} e={error:.3f} c={control:.3f} t={t:.3f} cy={yaw:.3f} cr={roll:.3f} cp={pitch_turn:.3f} gla={tlm.gps.latitude} glo={tlm.gps.longitude} glaz={tlm.gps.ground_course:.1f}")

            self._apply_attitude()

            time.sleep(0.05)

    def _height_loop(self):
        # Контур поддержания высоты. Управляет базовым pitch.
        while self.running and self.height_loop_running:
            tlm = self.controller.get_telemetry()

            # if tlm.gps.satellites < self.min_sats_for_gps:
            #     time.sleep(0.2)
            #     continue

            #current_alt = tlm.gps.altitude
            current_alt = self.zero_altitude + self.target_height - 50
            desired_alt = self.zero_altitude + self.target_height
            error = desired_alt - current_alt

            # Положительная ошибка (мы ниже цели) должна приводить к nose up (отрицательный pitch),
            # поэтому инвертируем знак.
            control = self._clamp(-error * self.height_kp, -1.0, 1.0)
            t = (control + 1.0) * 0.5
            pitch = self.lerp(self.pitch_height_min, self.pitch_height_max, t)

            with self._attitude_lock:
                self.base_pitch_angle = pitch

            log_print(f"H ca={current_alt:.1f} na={desired_alt:.1f} e={error:.3f} c={control:.3f} cp={pitch:.3f} gA={tlm.gps.altitude:.1f}")

            self._apply_attitude()
            time.sleep(0.05)

    def start_scenario_sync(self):
        self.running = True
        log_print("=== НАЧАЛО СЦЕНАРИЯ (HeroboecBlindAzimuth) ===")

        log_print("Инициализация...")
        self.controller.set_channel(0, 0)    # Roll
        self.controller.set_channel(1, 0)    # Pitch
        self.controller.set_channel(2, -1)   # Throttle (минимум)
        self.controller.set_channel(3, 0)    # Yaw
        self.controller.set_channel(4, 1)    # Arm
        self.controller.set_channel(5, 1)    # Mode ACRO (как в исходном сценарии)

        time.sleep(3.0)
        if not self.running:
            return

        # 0. Запоминаем текущую высоту как нулевую
        tlm = self.controller.get_telemetry()
        self.zero_altitude = tlm.gps.altitude
        self.zero_azimuth = tlm.angles.yaw
        log_print(f"Нулевая высота: {self.zero_altitude:.1f} м  Null azimuth={self.zero_azimuth:.1f}")

        # 1. Медленный подъем газа до половины
        log_print("Подъем газа до половины...")
        success = self.tween_alpha(
            self.to_half_throttle_time,
            lambda a: self.controller.set_channel(2, self.lerp(-1.0, self.half_throttle, a))
        )
        if not success:
            return

        log_print("Газ на половине")
        time.sleep(self.half_to_max_throttle_delay)
        if not self.running:
            return

        # 3. Быстрый подъем газа до максимума
        log_print("Подъем газа до максимума...")
        success = self.tween_alpha(
            self.to_max_throttle_time,
            lambda a: self.controller.set_channel(2, self.lerp(self.half_throttle, self.max_throttle, a))
        )
        if not success:
            return

        # 4. Взлет с максимальным газом
        log_print("Взлет...")
        time.sleep(self.liftoff_time)
        if not self.running:
            return

        # 5. Переход в режим Angle и установка pitch_from
        log_print("Переход в режим Angle...")
        self.controller.set_channel(5, 0)   # Mode ANGLE (предполагается, что 0 соответствует Angle)
        with self._attitude_lock:
            self.base_pitch_angle = self.pitch_from
            self.turn_pitch_offset = 0.0
            self.desired_roll = 0.0
            self.desired_yaw = 0.0
        self._apply_attitude()

        # 6. Короткая пауза после перехода в Angle
        time.sleep(self.to_angle_delay)
        if not self.running:
            return

        # 7. Запуск двух асинхронных контуров
        log_print("Запуск контуров по азимуту и высоте...")
        self._start_control_loops()

        # 8. Установка первых целевых значений
        if not self._tween_target_azimuth(self.program_azimuth1):
            return
        self.target_height = self.program_height1
        log_print(f"Цели #1: азимут {self.target_azimuth:.1f}°, высота {self.target_height:.1f} м от нуля")

        # 9. Пауза, пока контуры выруливают на цель
        time.sleep(self.program_1_2_pause)
        if not self.running:
            return
        # 10. Вторые целевые значения
        if not self._tween_target_azimuth(self.program_azimuth2):
            return
        self.target_height = self.program_height2
        log_print(f"Цели #2: азимут {self.target_azimuth:.1f}°, высота {self.target_height:.1f} м от нуля")

        # 11. Пауза
        time.sleep(self.program_2_3_pause)
        if not self.running:
            return

        # 12. Третьи целевые значения
        if not self._tween_target_azimuth(self.program_azimuth3):
            return
        self.target_height = self.program_height3
        log_print(f"Цели #3: азимут {self.target_azimuth:.1f}°, высота {self.target_height:.1f} м от нуля")

        # 13. Пауза
        time.sleep(self.program_3_4_pause)
        if not self.running:
            return

        # 14. Выключаем газ
        log_print("Выключение газа...")
        self.controller.set_channel(2, -1)

        log_print("=== СЦЕНАРИЙ ЗАВЕРШЕН ===")
        self.stop_all()


def wait_for_trigger(gpio_ctx, hold_time: float):
    """Ожидание, пока условие (PIN_26=1 ИЛИ удержание R) будет истинно hold_time секунд без разрыва."""

    interval = 0.05
    key_grace = 0.3  # сколько времени после последнего события R считаем, что R ещё "удерживается"

    line = None
    if gpio_ctx is not None:
        line = gpio_ctx[1]

    hold_since = None
    last_r_event = None

    while True:
        now = time.time()

        # --- Состояние пина ---
        pin_active = False
        if line is not None:
            try:
                value = line.get_value()
            except Exception:
                value = 0
            pin_active = (value == 1)

        # --- Состояние клавиши R (без Enter) ---
        key_event = False
        key_active = False

        stdin_ok = sys.stdin is not None and sys.stdin.isatty()
        if stdin_ok:
            if os.name == "nt":
                # Windows: используем msvcrt
                try:
                    import msvcrt
                    while msvcrt.kbhit():
                        ch = msvcrt.getch()
                        try:
                            c = ch.decode("utf-8", errors="ignore")
                        except Exception:
                            c = ""
                        if c.lower() == "r":
                            key_event = True
                except Exception:
                    pass
            else:
                # POSIX: termios + tty + select
                try:
                    import termios
                    import tty
                    fd = sys.stdin.fileno()
                    old_settings = termios.tcgetattr(fd)
                    try:
                        tty.setcbreak(fd)
                        rlist, _, _ = select.select([sys.stdin], [], [], 0)
                        if rlist:
                            ch = sys.stdin.read(1)
                            if ch.lower() == "r":
                                key_event = True
                    finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                except Exception:
                    pass

        if key_event:
            last_r_event = now

        if last_r_event is not None and (now - last_r_event) <= key_grace:
            key_active = True

        # Символы состояния
        if key_active:
            log_print("R", end="")
        if pin_active:
            log_print("P", end="")

        # --- Объединённое условие ---
        condition = pin_active or key_active

        if condition:
            if hold_since is None:
                hold_since = now
            else:
                elapsed = now - hold_since
                if elapsed >= hold_time:
                    if pin_active:
                        log_print("Триггер по GPIO PIN_26")
                        return "pin"
                    else:
                        log_print("Триггер по клавише R")
                        return "key"
        else:
            hold_since = None

        time.sleep(interval)


def on_telemetry(telemetry: CrsfTelemetryData):
    # Можно раскомментировать для отладки
    # print(f"Телеметрия: {telemetry}")
    pass


def main():
    init_logger()

    controller = CrsfController()
    controller.subscribe('all', on_telemetry)

    scenario = HeroboecBlindAzimuthScenario(controller)

    gpio_ctx = None
    if gpiod is not None and os.name != "nt":
        try:
            chip = gpiod.Chip("gpiochip3")
            line = chip.get_line(15)
            line.request(consumer="heroboec_blind_azimuth", type=gpiod.LINE_REQ_DIR_IN)
            gpio_ctx = (chip, line)
            log_print("GPIO PIN_26 доступен для запуска сценария")
        except Exception as e:
            log_print(f"Не удалось инициализировать GPIO: {e}")
            gpio_ctx = None

    try:
        controller.connect(
            port=scenario.port,
            baud_rate=420000,
            send_rate=20
        )

        log_print(f"\nОжидание сигнала запуска: логическая 1 на PIN_26 или клавиша R ({scenario.trigger_hold_time} секунд). Нажмите Ctrl+C для выхода.\n")

        while True:
            source = wait_for_trigger(gpio_ctx, scenario.trigger_hold_time)
            log_print("Запуск сценария HeroboecBlindAzimuth...")
            scenario.start_scenario_sync()
            log_print("Ожидание 60 секунд перед следующим запуском...")
            time.sleep(60.0)

    except KeyboardInterrupt:
        log_print("\n\nПрерывание пользователем")
    except Exception as e:
        log_print(f"Ошибка: {e}")
        import traceback
        traceback.print_exc()
        if _log_file is not None:
            try:
                traceback.print_exc(file=_log_file)
                _log_file.flush()
            except Exception:
                pass
    finally:
        scenario.stop_all()
        controller.disconnect()
        if gpio_ctx is not None:
            chip, line = gpio_ctx
            try:
                line.release()
            except Exception:
                pass
            try:
                chip.close()
            except Exception:
                pass
        log_print("Завершено")
        if _log_file is not None:
            try:
                _log_file.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
