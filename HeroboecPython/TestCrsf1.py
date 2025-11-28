# example_usage.py
import time
from CRSF import CrsfController, CrsfTelemetryData

def on_telemetry_received(telemetry: CrsfTelemetryData):
    """Callback для получения телеметрии"""
    print(f"Телеметрия: {telemetry}")

def main():
    # Создание контроллера
    controller = CrsfController()
    
    # Подписка на телеметрию
    controller.subscribe('all', on_telemetry_received)
    
    # Подключение к COM порту
    try:
        controller.connect(
            port='COM5',           # или '/dev/ttyUSB0' для Linux
            baud_rate=420000,
            send_rate=20
        )
        
        print("Подключено! Нажмите Ctrl+C для выхода")
        
        # Пример управления каналами
        # Установка значений каналов (0-15)
        # controller.set_channel(0, 0.5)   # Канал 0 = 50%
        # controller.set_channel(1, -0.3)  # Канал 1 = -30%
        # controller.set_channel(2, 0.0)   # Канал 2 = 0% (центр)
        # controller.set_channel(3, 1.0)   # Канал 3 = 100%
        
        # Основной цикл
        try:
            while True:
                # Здесь можно изменять каналы динамически
                # Например, синусоида на канале 0
                # t = time.time()
                # value = 0.5 * (1.0 + 0.5 * (t % 6.28))  # 0..1
                # controller.set_channel(0, value * 2 - 1)  # -1..1
                
                # Получение текущего значения канала
                # current_value = controller.get_channel(0)
                # print(f"Канал 0: {current_value:.3f}")
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\nПрерывание пользователем")
    
    except Exception as e:
        print(f"Ошибка: {e}")
    
    finally:
        # Отключение
        controller.disconnect()
        print("Завершено")

if __name__ == "__main__":
    main()