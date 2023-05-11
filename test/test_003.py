import cv2
from picamera2 import Picamera2
import time
import numpy as np
from fastiecm import fastiecm


def contrast_stretch(image):
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


def color_map(image):
    color_mapped_prep = image.astype(np.uint8)
    color_mapped_image = cv2.applyColorMap(color_mapped_prep, fastiecm)
    return color_mapped_image


redVal = 2.0
blueVal = 2.0


def RedSetting(val):
    redVal = val
    print("Red: ", redVal)


def BlueSetting(val):
    blueVal = val
    print("Blue: ", blueVal)


if __name__ == "__main__":
    dispW = 640
    dispH = 360
    picam2 = Picamera2()
    picam2.set_controls({"AwbEnable": 0})
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.main.size = (dispW, dispH)
    picam2.preview_configuration.controls.FrameRate = 20
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    time.sleep(2)

    while True:
        im = picam2.capture_array()
        with picam2.controls as controls:
            controls.ColourGains = (redVal, blueVal)
        contrast_im = contrast_stretch(im)
        ndvi_im = calc_ndvi(contrast_im)
        colour_map = color_map(ndvi_im)
        cv2.imshow("Camera", im)
        cv2.createTrackbar("Red", "Camera", 0, 32, RedSetting)
        cv2.createTrackbar("Blue", "Camera", 0, 32, BlueSetting)
        cv2.imshow("Contrasted", contrast_im)
        cv2.imshow("NDVI", ndvi_im)
        cv2.imshow("Mapped", colour_map)
        if cv2.waitKey(1) == ord("q"):
            break
    cv2.destroyAllWindows()
