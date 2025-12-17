import time
from typing import Optional

import serial.tools.list_ports

from CRSF import CrsfController
from TransformMath import ConvertQuaternionByBasis, Quaternion, Transform


def choose_com_port() -> Optional[str]:
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


def _fmt_vec3(v) -> str:
    return f"({v.x:.4f}, {v.y:.4f}, {v.z:.4f})"


def telemetry_quat_to_unity(q: Quaternion) -> Quaternion:
    m = (
        (0.0, 1.0, 0.0),
        (0.0, 0.0, -1.0),
        (-1.0, 0.0, 0.0),
    )
    inverse_rotation = False
    return ConvertQuaternionByBasis(q, m, inverse_rotation=inverse_rotation)


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

        print("\nПодключено. Печать forward/up/right 2 раза в секунду. Нажмите Ctrl+C для выхода.\n")

        while True:
            tlm = controller.get_telemetry()
            qx, qy, qz, qw = tlm.quaternion.rotation

            q_tel = Quaternion(qx, qy, qz, qw)
            q_unity = telemetry_quat_to_unity(q_tel)
            t = Transform(rotation=q_unity)

            forward = t.forward
            up = t.up
            right = t.right

            print(f"forward={_fmt_vec3(forward)} up={_fmt_vec3(up)} right={_fmt_vec3(right)}")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nОстановлено пользователем")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
