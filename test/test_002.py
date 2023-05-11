import cv2
from picamera2 import Picamera2
import time
import numpy as np

picam2 = Picamera2()

dispW = 640
dispH = 360
picam2.preview_configuration.main.size = (dispW, dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 15
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
fps = 0
pos = (30, 60)
font = cv2.FONT_HERSHEY_SIMPLEX
height = 1.5
weight = 3
myColor = (0, 0, 255)

hueLow = 70
hueHigh = 70
satLow = 170
satHigh = 255
valLow = 170
valHigh = 255


def onTrack1(val):
    hueLow = val
    print("Hue Low", hueLow)


def onTrack2(val):
    hueHigh = val
    print("Hue High", hueHigh)


def onTrack3(val):
    satLow = val
    print("Sat Low", satLow)


def onTrack4(val):
    satHigh = val
    print("Sat High", satHigh)


def onTrack5(val):
    valLow = val
    print("Val Low", valLow)


def onTrack6(val):
    valHigh = val
    print("Hue Low", valHigh)


def display(image, image_name):
    image = np.array(image, dtype=float) / float(255)
    shape = image.shape
    height = int(shape[0] / 2)
    width = int(shape[1] / 2)
    image = cv2.resize(image, (width, height))
    cv2.namedWindow(image_name)
    cv2.imshow(image_name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def contrast_stretch(im):
    in_min = np.percentile(im, 5)
    in_max = np.percentile(im, 95)

    out_min = 0.0
    out_max = 255.0

    out = im - in_min
    out *= (out_min - out_max) / (in_min - in_max)
    out += in_min

    return out


def contrast_stretch(im):
    in_min = np.percentile(im, 5)
    in_max = np.percentile(im, 95)

    out_min = 0.0
    out_max = 255.0

    out = im - in_min
    out *= (out_min - out_max) / (in_min - in_max)
    out += in_min

    return out


def calc_ndvi(image):
    b, g, r = cv2.split(image)
    bottom = r.astype(float) + b.astype(float)
    bottom[bottom == 0] = 0.01
    ndvi = (b.astype(float) - r) / bottom
    return ndvi


cv2.namedWindow("myTracker")

cv2.createTrackbar("Hue Low", "myTracker", 10, 179, onTrack1)
cv2.createTrackbar("Hue High", "myTracker", 30, 179, onTrack2)
cv2.createTrackbar("Sat Low", "myTracker", 100, 255, onTrack3)
cv2.createTrackbar("Sat High", "myTracker", 255, 255, onTrack4)
cv2.createTrackbar("Val Low", "myTracker", 100, 255, onTrack5)
cv2.createTrackbar("Val High", "myTracker", 255, 255, onTrack6)

while True:
    tStart = time.time()
    frame = picam2.capture_array()
    cv2.putText(frame, str(int(fps)) + " FPS", pos, font, height, myColor, weight)
    lowerBound = np.array([hueLow, satLow, valLow])
    upperBound = np.array([hueHigh, satHigh, valHigh])
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)
    myMaskSmall = cv2.resize(myMask, (int(dispW / 2), int(dispH / 2)))
    myObject = cv2.bitwise_and(frame, frame, mask=myMask)
    myObjectSmall = cv2.resize(myObject, (int(dispW / 2), int(dispH / 2)))
    cv2.imshow("Camera", frame)
    cv2.imshow("my Mask", myMaskSmall)
    cv2.imshow("My Objest", myObjectSmall)
    if cv2.waitKey(1) == ord("q"):
        break
    tEnd = time.time()
    loopTime = tEnd - tStart
    fps = 0.9 * fps + 0.1 * (1 / loopTime)
cv2.destroyAllWindows()
