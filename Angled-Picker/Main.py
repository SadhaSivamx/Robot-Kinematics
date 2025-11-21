import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import serial
import time

ARDUINO_PORT = 'COM13'
BAUD_RATE = 115200
TIMEOUT = 2
ser = None

Anglevalue = 90

try:
    print(f"Attempting to connect to {ARDUINO_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=TIMEOUT)
    time.sleep(2)
    ser.flushInput()
    print("Connection established.")

except serial.SerialException as e:
    print(f"Error connecting to Arduino: {e}")
    print("Continuing without serial communication.")

def speak2arduino(data):
    if ser is not None and ser.is_open:
        try:
            #taking
            messages=["M0:90","M1:90","M2:90","M4:90","M3:{}".format(int(Anglevalue)),"M5:0"]
            for message in messages:
                data_to_send = f"{message}\n".encode('utf-8')
                ser.write(data_to_send)
                print(f"Sent to Arduino: {message}")
                time.sleep(0.25)
            # placing
            messages = ["M3:{}".format(90),"M5:60"]
            for message in messages:
                data_to_send = f"{message}\n".encode('utf-8')
                ser.write(data_to_send)
                print(f"Sent to Arduino: {data}")
                time.sleep(0.25)

        except serial.SerialException as e:
            print(f"Serial write error: {e}")
    else:
        print("Serial port not open. Cannot send data.")

camidx = 0
cam = cv.VideoCapture(camidx, cv.CAP_DSHOW)

fontstyle = cv.FONT_HERSHEY_SIMPLEX
fontscale = 0.8
fcolor = (0, 255, 0)
thickness = 1

roi_selected = False
roi = None
def process_roi(frame, roi):
    (xstart, ystart, w, h) = roi

    currframe = frame[ystart:ystart+h, xstart:xstart+w]

    hsv_frame = cv.cvtColor(currframe, cv.COLOR_BGR2HSV)

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])

    mask = cv.inRange(hsv_frame, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.erode(mask, kernel, iterations=1)
    mask = cv.dilate(mask, kernel, iterations=1)

    binaryimg = mask

    contours, _ = cv.findContours(binaryimg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if not contours:
        return frame, None

    contours = sorted(contours, key=lambda x: cv.contourArea(x), reverse=True)
    maxcont = contours[0]

    if cv.contourArea(maxcont) < 50:
        return frame, None, None

    hull = cv.convexHull(maxcont)
    rect = cv.minAreaRect(hull)
    (cx, cy), (width, height), angle = rect
    center_x = int(cx) + xstart
    center_y = int(cy) + ystart
    center_point = (center_x, center_y)

    angle=(90-angle)
    if height>width:
        angle=90-(90-angle)
    else:
        angle=90+angle
    # draw rectangle
    box = cv.boxPoints(rect)
    box = np.int32(box + [xstart, ystart])
    cv.drawContours(frame, [box], 0, (0, 255, 0), 2)

    return frame, angle, center_point


while True:
    ret, frame = cam.read()
    if not ret:
        break

    frame = cv.resize(frame, (640, 480))

    key = cv.waitKey(1) & 0xFF

    # Select ROI once
    if key == ord('c'):
        cv.destroyWindow("Frame")
        roi = cv.selectROI("Select ROI", frame, showCrosshair=True, fromCenter=False)
        cv.destroyWindow("Select ROI")

        if roi[2] > 0 and roi[3] > 0:
            roi_selected = True
        else:
            roi_selected = False

    if key == ord('q'):
        break

    if key == ord('m'):
        message = "M3:{}".format(int(Anglevalue))
        print(message)
        speak2arduino(Anglevalue)

    if roi_selected:
        frame, angle, cp = process_roi(frame, roi)
        if angle is not None:
            Anglevalue=180-angle
            cv.circle(frame, cp, 3, (0, 0, 255), -1)
            cv.putText(frame, f"Angle: {angle:.2f}", (20, 450), fontstyle, fontscale, (0, 0, 255), thickness)

    cv.putText(frame, "Press C to set ROI, Q to quit",(20, 50), fontstyle, fontscale, fcolor, thickness)
    cv.imshow("Frame", frame)

cam.release()
cv.destroyAllWindows()
