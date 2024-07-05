import serial
import time
from pynput import keyboard

# Replace '/dev/ttyUSB0' with your Arduino's serial port if different
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

move = 0.0

def on_press(key):
    global move
    try:
        if key.char == 'a':
            move = -1.0
            ser.write(b'a')
            print(f'Sent move: {move}')
        elif key.char == 'd':
            move = 1.0
            ser.write(b'd')
            print(f'Sent move: {move}')
    except AttributeError:
        pass

def on_release(key):
    global move
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def main():
    print("Use 'a' and 'd' keys to control the stepper motor. Press 'esc' to quit.")
    # Collect events until released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    ser.close()

if __name__ == "__main__":
    main()
