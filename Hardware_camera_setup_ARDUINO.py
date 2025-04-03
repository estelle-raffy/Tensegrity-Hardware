## works with Camera_test_printing in Arduino IDE
## sends x,y coordinates to Arduino script
## Arduino has a single parsing function that reads and prints the coordinates
## Coordinates are read by Python and saved in Excel file --> whatever title 

from collections import deque
import numpy as np
import cv2
import imutils
import time
import serial  # Import pySerial for serial communication
import os
from openpyxl import Workbook  # For saving data to Excel

# Define the lower and upper boundaries of the "pink" ball in HSL
pinkLower = (150, 100, 50)
pinkUpper = (180, 255, 200)

# Buffer size for tracking points
buffer_size = 32
pts = deque(maxlen=buffer_size)
counter = 0
(dX, dY) = (0, 0)
direction = ""

# File setup
script_dir = os.path.dirname(os.path.abspath(__file__))
excel_file_path = os.path.join(script_dir, 'ExperimentsResults_Arduino-Python.xlsx')
length_log = []

# Establish serial connection to Arduino (update COM port)
try:
    ser = serial.Serial('COM7', 9600, timeout=1)  # Use correct port for your Arduino
    time.sleep(2)  # Allow serial connection to initialize
except serial.SerialException:
    print("[ERROR] Could not open serial port. Check the connection.")
    ser = None

# Start the video stream with Camera 0
print("[INFO] Starting video stream from external camera...")
vs = cv2.VideoCapture(0)

# Check if the camera is opened correctly
if not vs.isOpened():
    print("[ERROR] Could not open video stream from Camera 0.")
    exit()

time.sleep(2.0)  # Allow webcam to warm up

# Open Excel workbook
wb = Workbook()
ws = wb.active
ws.append(['Arduino_X', 'Arduino_Y'])

# Main loop to process the video frames
while True:
    # === 1️⃣ Receive latest data from Arduino ===
    if ser is not None and ser.in_waiting > 0:  
        try:
            arduino_data = ser.readline().decode('utf-8').strip()  # Read and clean data
            if "," in arduino_data:  # Ensure format is correct
                arduino_x, arduino_y = map(float, arduino_data.split(","))  # Convert to float
                print(f"[Arduino] X: {arduino_x}, Y: {arduino_y}")  # Debugging
                
                # ✅ Append data to Excel
                ws.append([arduino_x, arduino_y])

            else:
                print(f"[ERROR] Invalid data from Arduino: {arduino_data}")  # Handle bad data

        except ValueError:
            print(f"[ERROR] Could not convert data: {arduino_data}")  # Handle conversion errors

    # === 2️⃣ Track position in video ===
    ret, frame = vs.read()
    if not ret:
        print("[ERROR] Failed to capture frame.")
        break

    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsl = cv2.cvtColor(blurred, cv2.COLOR_BGR2HLS)
    mask = cv2.inRange(hsl, pinkLower, pinkUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            pts.appendleft(center)

            # === 3️⃣ Send latest (x, y) over serial ===
            if ser is not None:
                coords = f"{center[0]},{center[1]}\n"
                ser.write(coords.encode())  # Send coordinates to Arduino
                print(f"[Sent to Arduino] {coords.strip()}")

    # Show video output
    cv2.imshow("Tracking", frame)

    # === 4️⃣ Exit on 'q' press ===
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# === 5️⃣ Cleanup ===
vs.release()
cv2.destroyAllWindows()
if ser is not None:
    ser.close()  # Close serial connection
wb.save(excel_file_path)  # Save results to Excel
print("[INFO] Experiment finished. Data saved to Excel.")
