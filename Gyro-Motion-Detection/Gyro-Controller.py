import serial
import threading
import time
import keyboard

# ————— CONFIG —————
SERIAL_PORT = 'COM19'    # ← your port (e.g. 'COM3' or '/dev/ttyUSB0')
BAUD_RATE    = 230400
THR_X        = 5.0       # m/s² threshold for left/right
THR_Y        = 5.0       # m/s² threshold for up/down
PRESS_TIME   = 0.1       # seconds to hold each key

KEY_POS_X = 'down'
KEY_NEG_X = 'up'
KEY_POS_Y = 'right'
KEY_NEG_Y = 'left'

def press_and_release(key, duration=PRESS_TIME):
    t0 = time.strftime('%H:%M:%S')
    print(f"[{t0}] ➔ Pressing {key}")
    keyboard.press(key)
    time.sleep(duration)
    keyboard.release(key)
    t1 = time.strftime('%H:%M:%S')
    print(f"[{t1}]     Released {key}")

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Listening on {SERIAL_PORT} @ {BAUD_RATE} baud...")

    x_state = 0  # -1 = neg active, 0 = idle, +1 = pos active
    y_state = 0

    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        parts = line.split(',')
        # Expect exactly 2 values now (ax, ay)
        if len(parts) < 2:
            continue

        try:
            ax = float(parts[0])
            ay = float(parts[1])
        except ValueError:
            continue

        # Debug raw values
        # print(f"Raw → ax={ax:.2f}, ay={ay:.2f}")

        # ——— X Axis (Left/Right) ———
        if   ax >  THR_X and x_state !=  1:
            threading.Thread(target=press_and_release, args=(KEY_POS_X,)).start()
            x_state = 1
        elif ax < -THR_X and x_state != -1:
            threading.Thread(target=press_and_release, args=(KEY_NEG_X,)).start()
            x_state = -1
        elif -THR_X <= ax <= THR_X:
            x_state = 0

        # ——— Y Axis (Up/Down) ———
        if   ay >  THR_Y and y_state !=  1:
            threading.Thread(target=press_and_release, args=(KEY_POS_Y,)).start()
            y_state = 1
        elif ay < -THR_Y and y_state != -1:
            threading.Thread(target=press_and_release, args=(KEY_NEG_Y,)).start()
            y_state = -1
        elif -THR_Y <= ay <= THR_Y:
            y_state = 0

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting…")
