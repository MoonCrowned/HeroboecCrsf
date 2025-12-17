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

class HeroboecBlindAzimuthScenario:
    """Сценарий полета с поддержанием азимута и высоты по телеметрии"""

    def __init__(self, controller: CrsfController, config_path: Optional[str] = None):
        self.controller = controller
        self.running = False

        if config_path is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(base_dir, "HeroboecBlindAzimuth_config.json")

        self._load_config(config_path)

    def _load_config(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        self.port = str(cfg.get("port", "COM5"))

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

    def start_scenario_sync(self):
        self.running = True
        print("=== НАЧАЛО СЦЕНАРИЯ ПРОВЕРКИ ЭЛЕРОНОВ ===")

        print("Инициализация...")
        self.controller.set_channel(0, 0)    # Roll
        self.controller.set_channel(1, 0)    # Pitch
        self.controller.set_channel(2, -1)   # Throttle (минимум)
        self.controller.set_channel(3, 0)    # Yaw
        self.controller.set_channel(4, 1)    # Arm
        self.controller.set_channel(5, 1)    # Mode ACRO (как в исходном сценарии)

        time.sleep(3.0)
        if not self.running:
            return

        print("Pitch = 1")
        self.controller.set_channel(1, 1)
        time.sleep(2.0)

        print("Pitch = -1")
        self.controller.set_channel(1, -1)
        time.sleep(2.0)

        print("Yaw = 1")
        self.controller.set_channel(3, 1)
        time.sleep(2.0)

        print("Yaw = -1")
        self.controller.set_channel(3, -1)
        time.sleep(2.0)

        print("Roll = 1")
        self.controller.set_channel(0, 1)
        time.sleep(2.0)

        print("Roll = -1")
        self.controller.set_channel(0, -1)
        time.sleep(2.0)

        print("Roll = 0")
        self.controller.set_channel(0, 0)
        time.sleep(2.0)

        print("Relax")

        self.controller.set_channel(0, 0)    # Roll
        self.controller.set_channel(1, 0)    # Pitch
        self.controller.set_channel(2, -1)   # Throttle (минимум)
        self.controller.set_channel(3, 0)    # Yaw
        self.controller.set_channel(4, 1)    # Arm
        self.controller.set_channel(5, 1)

        time.sleep(0.5)

        print("=== СЦЕНАРИЙ ЗАВЕРШЕН ===")

def on_telemetry(telemetry: CrsfTelemetryData):
    # Можно раскомментировать для отладки
    # print(f"Телеметрия: {telemetry}")
    pass

def main():
    controller = CrsfController()
    controller.subscribe('all', on_telemetry)

    scenario = HeroboecBlindAzimuthScenario(controller)

    try:
        controller.connect(
            port=scenario.port,
            baud_rate=420000,
            send_rate=20
        )

        scenario.start_scenario_sync()

    except KeyboardInterrupt:
        print("\n\nПрерывание пользователем")
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        controller.disconnect()
        print("Завершено")

if __name__ == "__main__":
    main()
