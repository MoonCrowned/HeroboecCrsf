import time
import serial.tools.list_ports

from CRSF import CrsfController


def choose_com_port() -> str | None:
    """Показать нумерованный список COM-портов и вернуть выбранный порт (device)."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("COM-порты не найдены")
        return None

    print("Доступные COM-порты:")
    for idx, port in enumerate(ports, start=1):
        print(f"{idx}. {port.device}")

    while True:
        choice = input("Введите номер порта (или пустую строку для выхода): ").strip()
        if choice == "":
            return None
        try:
            num = int(choice)
        except ValueError:
            print("Некорректный ввод, нужно число.")
            continue

        if 1 <= num <= len(ports):
            return ports[num - 1].device
        else:
            print("Номер вне диапазона, попробуйте ещё раз.")


def main():
    port = choose_com_port()
    if port is None:
        print("Выход.")
        return

    controller = CrsfController()

    try:
        controller.connect(
            port=port,
            baud_rate=420000,
            send_rate=20,
        )

        print("\nПодключено. Печать телеметрии 2 раза в секунду. Нажмите Ctrl+C для выхода.\n")

        while True:
            tlm = controller.get_telemetry()
            print(tlm)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nОстановлено пользователем")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
