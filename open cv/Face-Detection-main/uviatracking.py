import cv2
import numpy as np
import serial
import time

# Initialize Serial Communication with Arduino (Update COM Port)
try:
    ser = serial.Serial("COM17", 115200, timeout=1)  # Change COM port if needed
    time.sleep(2)  # Allow time for serial connection
    print("Serial connection established.")
except serial.SerialException as e:
    print(f"Error: Could not open serial port - {e}")
    ser = None  # Avoid using 'ser' if it failed

# Use USB Webcam (Change index if needed)
camera_index = 1  # Try 2, 3, etc., if USB webcam is not detected
cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print(f"Error: Could not open USB webcam at index {camera_index}")
    exit
