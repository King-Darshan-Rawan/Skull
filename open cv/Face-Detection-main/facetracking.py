import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import serial  # Import serial for communication
import numpy as np

# Initialize Webcam
cap = cv2.VideoCapture(1)  # Use external webcam (0 for laptop cam)
ws, hs = 600, 600
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

# Initialize Arduino Board & Serial Communication
port = "COM17"
board = pyfirmata.Arduino(port)
ser = serial.Serial(port, 115200)  # Open Serial at 115200 baud

servo_pinX = board.get_pin('d:9:s')  # Pin 9 Arduino
servo_pinY = board.get_pin('d:10:s')  # Pin 10 Arduino

detector = FaceDetector()
servoPos = [90, 90]  # Initial servo position (centered)

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Get Face Coordinates
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]

        # Convert coordinates to servo degrees (0-180)
        servoX = np.interp(fx, [0, ws], [60, 120])
        servoY = np.interp(fy, [0, hs], [60, 120])

        # Ensure servo angles stay within valid range
        servoX = max(60, min(120, servoX))
        servoY = max(60, min(120, servoY))

        servoPos = [int(servoX), int(servoY)]

        # Send data to Arduino via Serial
        data_to_send = f"{servoPos[0]},{servoPos[1]}\n"
        print(f"Sending: {data_to_send.strip()}")  # Debugging Output
        ser.write(data_to_send.encode())  # Send as bytes

        # Draw UI Elements
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, f"X: {fx}, Y: {fy}", (fx + 15, fy - 15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)

    cv2.putText(img, f'Servo X: {servoPos[0]} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {servoPos[1]} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    # Move Servo Motors
    servo_pinX.write(servoPos[0])
    servo_pinY.write(servoPos[1])

    # Show the Video Feed
    cv2.imshow("Face Tracking", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
ser.close()
cap.release()
cv2.destroyAllWindows()
