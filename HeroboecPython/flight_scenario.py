# flight_scenario.py
import time
import math
from CRSF import CrsfController, CrsfTelemetryData

class FlightScenario:
    """Класс для выполнения сценария полета"""
    
    def __init__(self, controller: CrsfController):
        self.controller = controller
        self.running = False
        
        # Параметры сценария
        self.to_half_throttle_time = 1.0
        self.half_throttle = 0
        self.half_to_max_throttle_delay = 1.0  # секунды
        self.to_max_throttle_time = 1.0
        self.max_throttle = 1.0
        self.liftoff_time = 4.0  # секунды
        self.pitch_from = -0.5
        self.pitch_to = -0.05
        self.pitch_from_to_time = 2.0
        self.fly_time = 30.0
        self.fly_yaw_curve_factor = 0.15
    
    def stop_all(self):
        """Остановка сценария"""
        print("Остановка сценария...")
        self.running = False
        
        self.controller.set_channel(0, 0)    # Roll
        self.controller.set_channel(1, 0)    # Pitch
        self.controller.set_channel(2, -1)   # Throttle
        self.controller.set_channel(3, 0)    # Yaw
        
        time.sleep(0.5)
        
        self.controller.set_channel(4, 0)    # Disarm
        print("Остановлено")
    
    def tween_alpha(self, duration: float, callback, check_running=True):
        """
        Анимация значения от 0 до 1 за заданное время
        
        Args:
            duration: Длительность в секундах
            callback: Функция, принимающая alpha (0..1)
            check_running: Проверять ли флаг self.running
        """
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
            
            time.sleep(0.02)  # ~50 Hz обновление
        
        return True
    
    def lerp(self, a: float, b: float, t: float) -> float:
        """Линейная интерполяция"""
        return a + (b - a) * t
    
    def fly_yaw_curve(self, t: float) -> float:
        """
        Кривая для yaw во время полета
        Можно настроить под свои нужды
        """
        # Синусоида для плавных поворотов
        return math.sin(t * math.pi * 2) * self.fly_yaw_curve_factor
    
    async def start_scenario(self):
        """Запуск сценария полета"""
        self.running = True
        
        print("=== НАЧАЛО СЦЕНАРИЯ ===")
        
        # Инициализация
        print("Инициализация...")
        self.controller.set_channel(0, 0)    # Roll
        self.controller.set_channel(1, 0)    # Pitch
        self.controller.set_channel(2, -1)   # Throttle (минимум)
        self.controller.set_channel(3, 0)    # Yaw
        self.controller.set_channel(4, 1)    # Arm (армирование)
        self.controller.set_channel(5, 1)    # Mode ACRO
        
        time.sleep(3.0)
        if not self.running:
            return
        
        # Плавный подъем газа до половины
        print("Подъем газа до половины...")
        success = self.tween_alpha(
            self.to_half_throttle_time,
            lambda a: self.controller.set_channel(2, self.lerp(-1.0, self.half_throttle, a))
        )
        if not success:
            return
        
        print("Газ на половине")
        time.sleep(self.half_to_max_throttle_delay)
        if not self.running:
            return
        
        # Подъем газа до максимума
        print("Подъем газа до максимума...")
        success = self.tween_alpha(
            self.to_max_throttle_time,
            lambda a: self.controller.set_channel(2, self.lerp(self.half_throttle, self.max_throttle, a))
        )
        if not success:
            return
        
        # Время на взлет
        print("Взлет...")
        time.sleep(self.liftoff_time)
        if not self.running:
            return
        
        # Переключение в режим Horizon
        print("Переключение в режим Horizon...")
        self.controller.set_channel(5, 0)  # Mode Horizon
        self.controller.set_channel(1, self.pitch_from)  # Pitch
        time.sleep(0.3)
        if not self.running:
            return
        
        # Плавное изменение pitch к горизонту
        print("Выравнивание pitch...")
        success = self.tween_alpha(
            self.pitch_from_to_time,
            lambda a: self.controller.set_channel(1, self.lerp(self.pitch_from, self.pitch_to, a))
        )
        if not success:
            return
        
        # Маневры по yaw
        print("Маневры по yaw...")
        success = self.tween_alpha(
            self.fly_time,
            lambda a: self.controller.set_channel(3, self.fly_yaw_curve(a * 4))
        )
        if not success:
            return
        
        print("=== СЦЕНАРИЙ ЗАВЕРШЕН ===")
        self.stop_all()
    
    def start_scenario_sync(self):
        """Синхронная версия (без async)"""
        self.running = True
        
        print("=== НАЧАЛО СЦЕНАРИЯ ===")
        
        # Инициализация
        print("Инициализация...")
        self.controller.set_channel(0, 0)    # Roll
        self.controller.set_channel(1, 0)    # Pitch
        self.controller.set_channel(2, -1)   # Throttle (минимум)
        self.controller.set_channel(3, 0)    # Yaw
        self.controller.set_channel(4, 1)    # Arm (армирование)
        self.controller.set_channel(5, 1)    # Mode ACRO
        
        time.sleep(3.0)
        if not self.running:
            return
        
        # Плавный подъем газа до половины
        print("Подъем газа до половины...")
        success = self.tween_alpha(
            self.to_half_throttle_time,
            lambda a: self.controller.set_channel(2, self.lerp(-1.0, self.half_throttle, a))
        )
        if not success:
            return
        
        print("Газ на половине")
        time.sleep(self.half_to_max_throttle_delay)
        if not self.running:
            return
        
        # Подъем газа до максимума
        print("Подъем газа до максимума...")
        success = self.tween_alpha(
            self.to_max_throttle_time,
            lambda a: self.controller.set_channel(2, self.lerp(self.half_throttle, self.max_throttle, a))
        )
        if not success:
            return
        
        # Время на взлет
        print("Взлет...")
        time.sleep(self.liftoff_time)
        if not self.running:
            return
        
        # Переключение в режим Horizon
        print("Переключение в режим Horizon...")
        self.controller.set_channel(5, 0)  # Mode Horizon
        self.controller.set_channel(1, self.pitch_from)  # Pitch
        time.sleep(0.3)
        if not self.running:
            return
        
        # Плавное изменение pitch к горизонту
        print("Выравнивание pitch...")
        success = self.tween_alpha(
            self.pitch_from_to_time,
            lambda a: self.controller.set_channel(1, self.lerp(self.pitch_from, self.pitch_to, a))
        )
        if not success:
            return
        
        # Маневры по yaw
        print("Маневры по yaw...")
        success = self.tween_alpha(
            self.fly_time,
            lambda a: self.controller.set_channel(3, self.fly_yaw_curve(a * 4))
        )
        if not success:
            return
        
        print("=== СЦЕНАРИЙ ЗАВЕРШЕН ===")
        self.stop_all()


def on_telemetry(telemetry: CrsfTelemetryData):
    """Callback для телеметрии"""
    #print(f"Телеметрия: {telemetry}")
    pass

def main():
    # Создание контроллера
    controller = CrsfController()
    
    # Подписка на телеметрию
    controller.subscribe('all', on_telemetry)
    
    # Создание сценария
    scenario = FlightScenario(controller)
    
    try:
        # Подключение
        controller.connect(
            port='COM5',
            baud_rate=420000,
            send_rate=20
        )
        
        print("\n=== УПРАВЛЕНИЕ ===")
        print("Введите '1' и нажмите Enter для запуска сценария")
        print("Нажмите Ctrl+C для выхода")
        print("==================\n")
        
        while True:
            command = input("Команда: ").strip()
            
            if command == '1':
                print("\nЗапуск сценария полета...")
                scenario.start_scenario_sync()
            elif command == '0':
                print("\nОстановка...")
                scenario.stop_all()
            elif command.lower() in ['exit', 'quit', 'q']:
                print("\nВыход...")
                break
            else:
                print("Неизвестная команда. Используйте '1' для запуска")
    
    except KeyboardInterrupt:
        print("\n\nПрерывание пользователем")
    
    except Exception as e:
        print(f"Ошибка: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        scenario.stop_all()
        controller.disconnect()
        print("Завершено")


if __name__ == "__main__":
    main()