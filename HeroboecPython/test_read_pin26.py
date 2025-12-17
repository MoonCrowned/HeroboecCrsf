#!/usr/bin/env python3
import gpiod
import time
import sys

CHIP_NAME = "gpiochip3"   # как в gpioinfo
LINE_OFFSET = 15          # PIN_26
SLEEP_INTERVAL = 0.2      # 5 раз в секунду

def main():
    # Открываем чип
    chip = gpiod.Chip(CHIP_NAME)

    # Берём линию PIN_26
    line = chip.get_line(LINE_OFFSET)

    # Запрашиваем линию как вход
    line.request(
        consumer="gpio_reader",
        type=gpiod.LINE_REQ_DIR_IN
    )

    try:
        while True:
            value = line.get_value()  # 0 или 1
            sys.stdout.write(str(value))
            sys.stdout.flush()
            time.sleep(SLEEP_INTERVAL)
    except KeyboardInterrupt:
        print()  # перевод строки при Ctrl-C
    finally:
        line.release()
        chip.close()

if __name__ == "__main__":
    main()