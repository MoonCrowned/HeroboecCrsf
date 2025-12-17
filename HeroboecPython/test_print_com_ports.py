import serial.tools.list_ports

def list_com_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("COM-порты не найдены")
        return

    print("Доступные COM-порты:")
    for port in ports:
        # port.device — имя порта (например, COM3 или /dev/ttyUSB0)
        print(port.device)

if __name__ == "__main__":
    list_com_ports()