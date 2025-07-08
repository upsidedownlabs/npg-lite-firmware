#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.

#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <https://www.gnu.org/licenses/>.

#  Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
#  Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

#  At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
#  Our mission is to make neuroscience affordable and accessible for everyone.
#  By supporting us with your purchase, you help spread innovation and open science.
#  Thank you for being part of this journey with us!

import serial
import threading
import time
import keyboard

# ————— CONFIG —————
SERIAL_PORT = 'COM12'    # ← your port (e.g. 'COM3' or '/dev/ttyUSB0')
BAUD_RATE    = 230400
THR_X        = 5.0       # m/s² threshold for left/right
THR_Y        = 5.0       # m/s² threshold for up/down
THR_Z        = 5.0
PRESS_TIME   = 0.1       # seconds to hold each key

KEY_POS_X = 'down'
KEY_NEG_X = 'up'
KEY_POS_Y = 'right'
KEY_NEG_Y = 'left'
KEY_POS_Z  = 'page_down'
KEY_NEG_Z  = 'page_up'

def press_and_release(key, duration=PRESS_TIME):
    t0 = time.strftime('%H:%M:%S')
    print(f"[{t0}] ➔ Pressing {key}")
    keyboard.press(key)
    time.sleep(duration)
    keyboard.release(key)
    t1 = time.strftime('%H:%M:%S')
    print(f"[{t1}]     Released {key}")

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}: {e}")
        return
    print(f"Listening on {SERIAL_PORT} @ {BAUD_RATE} baud...")

    x_state = 0  # -1 = neg active, 0 = idle, +1 = pos active
    y_state = 0
    z_state = 0

    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        parts = line.split(',')
        # Expect exactly 3 values now (ax, ay, az)
        if len(parts) < 3:
            continue

        try:
            ax = float(parts[0])
            ay = float(parts[1])
            az = float(parts[2])
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

        # # ——— Z Axis ———
        # if   az >  THR_Z and z_state !=  1:
        #     threading.Thread(target=press_and_release, args=(KEY_POS_Z,)).start()
        #     z_state = 1
        # elif az < -THR_Z and z_state != -1:
        #     threading.Thread(target=press_and_release, args=(KEY_NEG_Z,)).start()
        #     z_state = -1
        # elif -THR_Z <= az <= THR_Z:
        #     z_state = 0

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting…")
