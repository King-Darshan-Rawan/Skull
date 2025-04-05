import cv2
import numpy as np
import serial
import time

# Initialize Serial Communication with Arduino
ser = serial.Serial("COM17", 115200, timeout=1)  # Change COM port if needed
time.sleep(2)  # Allow time for serial connection

# Open the webcam
cap = cv2.VideoCapture(0)
ws, hs = 640, 480  # Camera resolution

# Face Detector
detector = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# Initial servo position
servoX = 90  # Y is fixed at 90

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

        print(f"Face Coordinates: X = {face_center[0]}")

        # Convert pixel X-coordinate to servo angle (only X moves)
        servoX = int(np.interp(face_center[0], [0, ws], [0, 180]))  # Adjusted X range

        # Send X-axis data only (Y is fixed at 90)
        data_to_send = f"{servoX},90\n"  
        print(f"Sending: {data_to_send.strip()}")
        ser.write(data_to_send.encode())

        # Draw detection box
        cv2.rectangle(img, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
        cv2.putText(img, f"X: {servoX}", (fx, fy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, "TARGET LOCKED", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    else:
        cv2.putText(img, "NO TARGET", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Face Tracking", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
cap.release()
cv2.destroyAllWindows()
