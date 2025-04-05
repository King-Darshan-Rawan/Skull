import cv2
import numpy as np
import serial
import time

# Set ESP32-CAM video stream URL
ESP32_STREAM_URL = "http://192.168.102.116:81/stream"  # Replace with actual IP
cap = cv2.VideoCapture(ESP32_STREAM_URL)

# Connect to ESP32 Serial (COM15)
ser = serial.Serial("COM15", 115200, timeout=1)
time.sleep(2)  # Wait for serial to initialize

# Face Detector
detector = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

ws, hs = 640, 480  # Video resolution
servoPos = [90, 90]  # Initial servo positions

while True:
    success, img = cap.read()
    if not success:
        print("Failed to get video feed!")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = detector.detectMultiScale(gray, 1.3, 5)

    if len(faces) > 0:
        (fx, fy, fw, fh) = faces[0]
        face_center = (fx + fw // 2, fy + fh // 2)

        # Print coordinates of detected face
        print(f"Face Coordinates: X = {face_center[0]}, Y = {face_center[1]}")

        # Convert coordinates to servo degrees
        servoX = np.interp(face_center[0], [0, ws], [0, 180])
        servoY = np.interp(face_center[1], [0, hs], [0, 180])

        servoPos = [int(servoX), int(servoY)]
        ser.write(f"{servoPos[0]},{servoPos[1]}\n".encode())  # Send to ESP32

        # Draw on image
        cv2.rectangle(img, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
        cv2.putText(img, f"X: {face_center[0]}, Y: {face_center[1]}", (fx, fy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, "TARGET LOCKED", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    else:
        cv2.putText(img, "NO TARGET", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("ESP32-CAM", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
cap.release()
cv2.destroyAllWindows()
