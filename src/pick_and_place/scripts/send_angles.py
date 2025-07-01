#!/usr/bin/env python3

import sys
import json
import serial
import time

# Read the JSON string passed from C++
if len(sys.argv) < 2:
    print("No data received.")
    sys.exit(1)

data = sys.argv[1]

try:
    # Open serial connection to STM (adjust port and baud rate as needed)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
    time.sleep(2)  # Wait for STM to initialize

    # Send data
    ser.write((data + '\n').encode('utf-8'))
    print(f"Sent: {data}")

    # Wait for acknowledgment from STM
    ack = ser.readline().decode('utf-8').strip()
    print(f"Received: {ack}")

    if ack == "ACK":
        sys.exit(0)  # Success
    else:
        sys.exit(2)  # Bad or no ACK

except Exception as e:
    print(f"Serial communication error: {e}")
    sys.exit(1)
