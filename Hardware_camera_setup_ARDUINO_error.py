from collections import deque
import numpy as np
import cv2
import imutils
import time
import serial  # Import pySerial for serial communication
import os
import math  # For Euclidean distance calculation
from openpyxl import Workbook  # For saving data to Excel

# Define the lower and upper boundaries of the "pink" ball in HSL
pinkLower = (150, 100, 50)
pinkUpper = (180, 255, 200)

# Buffer size for tracking points
buffer_size = 32
pts = deque(maxlen=buffer_size)
counter = 0
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
ws.append(['Error_Distance'])  # Save only error values

# Main loop to process the video frames
while True:
    # === 1️⃣ Receive latest data from Arduino ===
    if ser is not None and ser.in_waiting > 0:
        arduino_data = ser.readline().decode('utf-8').strip()  # Read from Arduino
        print(f"[Arduino] {arduino_data}")  # Debug print

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

    # === 3️⃣ Get Object Position and Compute Error ===
    frame_height, frame_width, _ = frame.shape  # Get frame size
    target_x, target_y = frame_width // 2, frame_height // 2  # Center position (target)

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            pts.appendleft(center)

            # Compute Euclidean error distance
            error_distance = math.sqrt((target_x - center[0])**2 + (target_y - center[1])**2)
            print(f"[Computed Error] Distance: {error_distance:.2f} pixels")

            # === 4️⃣ Send error to Arduino ===
            if ser is not None:
                error_msg = f"{error_distance:.2f}\n"
                ser.write(error_msg.encode())  # Send error to Arduino
                print(f"[Sent to Arduino] {error_msg.strip()}")

            # Save error to Excel
            ws.append([error_distance])

    # === 5️⃣ Draw Virtual Target at Center ===
    cross_size = 20  # Length of cross arms in pixels
    cross_color = (0, 255, 0)  # Green color
    thickness = 2  # Line thickness

    cv2.line(frame, (target_x - cross_size, target_y), (target_x + cross_size, target_y), cross_color, thickness)
    cv2.line(frame, (target_x, target_y - cross_size), (target_x, target_y + cross_size), cross_color, thickness)
    cv2.putText(frame, "Target", (target_x + 10, target_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cross_color, 2)

    # Show video output
    cv2.imshow("Tracking", frame)

    # === 6️⃣ Exit on 'q' press ===
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# === 7️⃣ Cleanup ===
vs.release()
cv2.destroyAllWindows()
if ser is not None:
    ser.close()  # Close serial connection
wb.save(excel_file_path)  # Save results to Excel
print("[INFO] Experiment finished. Data saved to Excel.")
