# Import required packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import serial  # Import pySerial for serial communication

# Define the lower and upper boundaries of the "pink" ball in HSV
pinkLower = (140, 100, 100)  
pinkUpper = (170, 255, 255)  

# Buffer size for tracking points
buffer_size = 32
pts = deque(maxlen=buffer_size)
counter = 0
(dX, dY) = (0, 0)
direction = ""

# Establish serial connection to Arduino (replace 'COM3' with your Arduino port)
ser = serial.Serial('COM7', 9600)  # Replace 'COM3' with the correct port (use 'ls /dev/tty*' on Linux or 'Device Manager' on Windows)

# Start the webcam video stream
print("[INFO] Starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)  # Allow webcam to warm up

# Main loop to process the video frames
while True:
    # Capture frame from the webcam
    frame = vs.read()
    
    # Resize the frame for faster processing
    frame = imutils.resize(frame, width=600)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Create a mask to detect only pink colors
    mask = cv2.inRange(hsv, pinkLower, pinkUpper)
    
    # Remove small noise (erosion + dilation)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # Only proceed if at least one contour is found
    if len(cnts) > 0:
        # Find the largest contour
        c = max(cnts, key=cv2.contourArea)
        
        # Get the circle around the detected object
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        
        # Compute the center using image moments
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Only proceed if the radius is large enough
        if radius > 10:
            # Draw the circle and center on the frame
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            pts.appendleft(center)

            # Send coordinates to Arduino via serial
            if center is not None:
                # Format the coordinates into a string for Arduino
                coords = f"{center[0]},{center[1]}\n"
                ser.write(coords.encode())  # Send the coordinates to Arduino

                # Optionally, print coordinates for debugging
                print(f"Sent to Arduino: {coords}")

    # Loop over the tracking points
    for i in range(1, len(pts)):
        if pts[i - 1] is None or pts[i] is None:
            continue

        # Compute direction of movement
        if counter >= 10 and i == 1 and pts[-10] is not None:
            dX = pts[-10][0] - pts[i][0]
            dY = pts[-10][1] - pts[i][1]
            dirX, dirY = "", ""

            if abs(dX) > 20:
                dirX = "Right" if np.sign(dX) == 1 else "Left"

            if abs(dY) > 20:
                dirY = "Up" if np.sign(dY) == 1 else "Down"

            direction = f"{dirY}-{dirX}" if dirX and dirY else dirX or dirY

        # Draw tracking lines
        thickness = int(np.sqrt(buffer_size / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # Display movement direction and coordinates
    cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)
    cv2.putText(frame, f"dx: {dX}, dy: {dY}", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    
    # Display the coordinates of the pink object in real-time on the screen
    if center is not None:
        cv2.putText(frame, f"X: {center[0]}, Y: {center[1]}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

    # Show the output frame
    cv2.imshow("Tracking", frame)
    
    # Break loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Stop the video stream and close windows
vs.stop()
cv2.destroyAllWindows()
ser.close()  # Close the serial connection after finishing
