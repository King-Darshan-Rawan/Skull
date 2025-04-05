import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time  # Add time module for delays

# Initialize Camera
cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

# Initialize Arduino and Servos
port = "COM17"
board = pyfirmata.Arduino(port)
servo_pinX = board.get_pin('d:9:s')   # Pin 9 for Servo X
servo_pinY = board.get_pin('d:11:s')  # Pin 11 for Servo Y

board.iterate()  # Important for PyFirmata communication
time.sleep(2)  # Give some time to establish connection

# Face Detector
detector = FaceDetector()
servoPos = [90, 90]  # Initial servo positions (center)

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Get face center coordinates
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        
        # Convert coordinates to servo angles
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [0, 180])

        # Limit servo values to valid range (0-180 degrees)
        servoX = max(0, min(180, servoX))
        servoY = max(0, min(180, servoY))

        servoPos = [servoX, servoY]

        # Draw tracking visuals
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str((fx, fy)), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)

    # Display servo position on screen
    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    # Move Servos
    servo_pinX.write(servoPos[0])
    servo_pinY.write(servoPos[1])
    time.sleep(0.05)  # Small delay to
