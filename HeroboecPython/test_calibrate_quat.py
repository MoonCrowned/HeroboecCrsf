import itertools
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import serial.tools.list_ports

from CRSF import CrsfController
from TransformMath import ConvertQuaternionByBasis, Mat3, Quaternion, Transform, Vector3


@dataclass
class PoseSample:
    q_tel: Quaternion
    expected_forward: Vector3
    expected_up: Vector3
    expected_right: Vector3


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


def _fmt_vec3(v: Vector3) -> str:
    return f"({v.x:.4f}, {v.y:.4f}, {v.z:.4f})"


def _basis_matrices() -> List[Mat3]:
    axes = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    ]

    mats: List[Mat3] = []
    for perm in itertools.permutations(axes, 3):
        for sx, sy, sz in itertools.product([1.0, -1.0], repeat=3):
            m0 = (perm[0][0] * sx, perm[0][1] * sx, perm[0][2] * sx)
            m1 = (perm[1][0] * sy, perm[1][1] * sy, perm[1][2] * sy)
            m2 = (perm[2][0] * sz, perm[2][1] * sz, perm[2][2] * sz)
            mats.append((m0, m1, m2))

    uniq: List[Mat3] = []
    seen = set()
    for m in mats:
        key = tuple(round(v, 0) for row in m for v in row)
        if key in seen:
            continue
        seen.add(key)
        uniq.append(m)
    return uniq


def _score_sample(q_unity: Quaternion, expected_forward: Vector3, expected_up: Vector3, expected_right: Vector3) -> float:
    t = Transform(rotation=q_unity)
    f = t.forward.normalized
    u = t.up.normalized
    r = t.right.normalized

    ef = expected_forward.normalized
    eu = expected_up.normalized
    er = expected_right.normalized

    df = 1.0 - Vector3.Dot(f, ef)
    du = 1.0 - Vector3.Dot(u, eu)
    dr = 1.0 - Vector3.Dot(r, er)

    return df * df + du * du + dr * dr


def find_best_mapping(samples: List[PoseSample]) -> Tuple[Mat3, bool, float]:
    mats = _basis_matrices()

    best_m = mats[0]
    best_inv = False
    best_score = 1e9

    for inv in (False, True):
        for m in mats:
            total = 0.0
            for s in samples:
                qu = ConvertQuaternionByBasis(s.q_tel, m, inverse_rotation=inv)
                total += _score_sample(qu, s.expected_forward, s.expected_up, s.expected_right)
            if total < best_score:
                best_score = total
                best_m = m
                best_inv = inv

    return best_m, best_inv, best_score


def _read_quat(controller: CrsfController) -> Quaternion:
    tlm = controller.get_telemetry()
    qx, qy, qz, qw = tlm.quaternion.rotation
    return Quaternion(qx, qy, qz, qw)


def _prompt_pose() -> Tuple[Vector3, Vector3, Vector3]:
    print("Введите ожидаемую позу в Unity-виде (вектора базиса в мировых координатах).")
    print("Формат: три числа через пробел. Пример: 0 0 1")

    def read_vec(name: str) -> Vector3:
        while True:
            s = input(f"{name}: ").strip()
            parts = s.split()
            if len(parts) != 3:
                print("Нужно ровно 3 числа")
                continue
            try:
                x, y, z = (float(parts[0]), float(parts[1]), float(parts[2]))
            except ValueError:
                print("Некорректные числа")
                continue
            return Vector3(x, y, z)

    expected_forward = read_vec("expected_forward")
    expected_up = read_vec("expected_up")
    expected_right = read_vec("expected_right")
    return expected_forward, expected_up, expected_right


def _mat3_to_text(m: Mat3) -> str:
    return (
        f"(({int(m[0][0])}, {int(m[0][1])}, {int(m[0][2])}), "
        f"({int(m[1][0])}, {int(m[1][1])}, {int(m[1][2])}), "
        f"({int(m[2][0])}, {int(m[2][1])}, {int(m[2][2])}))"
    )


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

        print("\nПодключено.")
        print("Скрипт собирает несколько поз (q_telemetry + ожидаемые forward/up/right в Unity) и подбирает преобразование.")
        print("Рекомендуется минимум 3-4 позы.\n")

        samples: List[PoseSample] = []

        while True:
            cmd = input("Команда: [enter]=снять позу, f=закончить, q=выход: ").strip().lower()
            if cmd == "q":
                return
            if cmd == "f":
                break

            print("Ждём свежую телеметрию кватерниона...")
            time.sleep(0.2)
            q_tel = _read_quat(controller).normalized

            print(f"q_tel={q_tel}")
            expected_forward, expected_up, expected_right = _prompt_pose()

            samples.append(
                PoseSample(
                    q_tel=q_tel,
                    expected_forward=expected_forward,
                    expected_up=expected_up,
                    expected_right=expected_right,
                )
            )

            print(f"Сэмпл добавлен. forward={_fmt_vec3(expected_forward)} up={_fmt_vec3(expected_up)} right={_fmt_vec3(expected_right)}")

        if not samples:
            print("Нет сэмплов. Выход.")
            return

        best_m, best_inv, best_score = find_best_mapping(samples)

        print("\n--- Result ---")
        print(f"basis_matrix M = {_mat3_to_text(best_m)}")
        print(f"inverse_rotation = {best_inv}")
        print(f"score = {best_score:.6f}")

        print("\nПроверка на последнем сэмпле:")
        qu = ConvertQuaternionByBasis(samples[-1].q_tel, best_m, inverse_rotation=best_inv)
        t = Transform(rotation=qu)
        print(f"forward={_fmt_vec3(t.forward)}")
        print(f"up     ={_fmt_vec3(t.up)}")
        print(f"right  ={_fmt_vec3(t.right)}")

        print("\nДальше в рантайме используй:")
        print("from TransformMath import ConvertQuaternionByBasis")
        print("q_unity = ConvertQuaternionByBasis(q_tel, M, inverse_rotation=...)\n")

    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
