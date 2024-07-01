import serial
import time

def read_floats_from_serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    data = float(line)
                    print(f'Received: {data}')
            except ValueError:
                print('Failed to parse float from received data')
            except KeyboardInterrupt:
                print('Exiting...')
                break
    finally:
        ser.close()
        print('Serial port closed')

if __name__ == '__main__':
    read_floats_from_serial()
