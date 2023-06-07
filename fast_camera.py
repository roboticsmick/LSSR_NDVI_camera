import cv2
import os
import time
from picamera2 import Picamera2
import numpy as np
from fastiecm import fastiecm
from pathlib import Path
import RPi.GPIO as GPIO

# Pin definitions
BUTTON_01 = 4
LED_PIN1 = 17
LED_PIN2 = 27
LED_PIN3 = 22
LED_PIN4 = 10
LED_PIN5 = 24
LED_PIN6 = 23

# Use "GPIO" pin numbering
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Set button pin as input
GPIO.setup(BUTTON_01, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set LED pin as output
GPIO.setup(LED_PIN1, GPIO.OUT)
GPIO.setup(LED_PIN2, GPIO.OUT)
GPIO.setup(LED_PIN3, GPIO.OUT)
GPIO.setup(LED_PIN4, GPIO.OUT)
GPIO.setup(LED_PIN5, GPIO.OUT)
GPIO.setup(LED_PIN6, GPIO.OUT)

# Set LED pin as output
GPIO.output(LED_PIN1, GPIO.LOW)
GPIO.output(LED_PIN2, GPIO.LOW)
GPIO.output(LED_PIN3, GPIO.LOW)
GPIO.output(LED_PIN4, GPIO.LOW)
GPIO.output(LED_PIN5, GPIO.LOW)
GPIO.output(LED_PIN6, GPIO.LOW)

# Initialise camera variables
REDVAL = 0.544
REDINIT = int(REDVAL / (32 / 255))
BLUEVAL = 1.248
BLUEINIT = int(BLUEVAL / (32 / 255))
AGAINVAL = 9.286
AGINIT = int(AGAINVAL / (10 / 255))
CONTRAST = 1.376
CINIT = int(CONTRAST / (2 / 255))
BRIGHT = 0.0
BINIT = int(255 / 2)

# Set up variables for the images folder and filenames
base_folder = Path(__file__).parent.resolve()

# Set up variables for the camera state
CAMERA_CAPTURE = False  # Flag variable to track photo capture state
CAPTURE_INTERVAL = 0.2  # Interval between capturing photos (5 photos per second)


# Analogue gain red slider setting
def RedSetting(val):
    global REDVAL
    redFloat = val * (32 / 255)
    REDVAL = redFloat
    print("Red: ", REDVAL)


# Analogue gain blue slider setting
def BlueSetting(val):
    global BLUEVAL
    blueFloat = val * (32 / 255)
    BLUEVAL = blueFloat
    print("Blue: ", blueFloat)


# Analogue gain slider setting
def AnalogueGainSetting(val):
    global AGAINVAL
    AGainFloat = val * (10 / 255)
    AGAINVAL = AGainFloat
    print("Analogue Gain: ", AGainFloat)


# Contrast slider setting
def ContrastSetting(val):
    global CONTRAST
    ContrastFloat = val * (2 / 255)
    CONTRAST = ContrastFloat
    print("Contrast: ", ContrastFloat)


# Brightness slider setting
def BrightnessSetting(val):
    global BRIGHT
    BrightFloat = (val / 255) * 2 - 1
    BRIGHT = BrightFloat
    print("Brightness: ", BrightFloat)


# Print camera settings
def PrintSettings():
    metadata = picam2.capture_metadata()
    controls = {c: metadata[c] for c in ["ExposureTime", "AnalogueGain", "ColourGains"]}
    print(picam2.camera_controls["ExposureTime"])
    print(picam2.camera_controls["AnalogueGain"])
    print(picam2.camera_controls["ColourGains"])


# Image contrast stretch
def contrast_stretch(image):
    in_min = np.percentile(image, 5)
    in_max = np.percentile(image, 95)
    out_min = 0.0
    out_max = 255.0
    out = image - in_min
    out *= (out_min - out_max) / (in_min - in_max)
    out += in_min
    return out


# Filter NDVI
def calc_ndvi(image):
    b, g, r, a = cv2.split(image)
    bottom = r.astype(float) + b.astype(float)
    bottom[bottom == 0] = 0.01
    ndvi = (b.astype(float) - r) / bottom
    return ndvi


# Colour map NDVI
def color_map(image):
    color_mapped_prep = image.astype(np.uint8)
    color_mapped_image = cv2.applyColorMap(color_mapped_prep, fastiecm)
    return color_mapped_image


if __name__ == "__main__":
    picam2 = Picamera2()
    picam2.set_controls(
        {"ExposureTime": 12000, "AnalogueGain": 1.2, "AwbEnable": 0, "AeEnable": 0}
    )
    # picam2.set_controls({"ExposureTime": 12000, "AnalogueGain": AGAINVAL, "AwbEnable": 0, "AeEnable": 0, "ColourGains": (REDVAL, BLUEVAL), "Contrast": CONTRAST, "Brightness": BRIGHT})
    picam2.preview_configuration.main.format = "XBGR8888"
    picam2.preview_configuration.main.size = (640, 360)
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.start()
    time.sleep(2)

    # Set up variables for the images folder and filenames
    images_folder = "fast_images"
    image_name_prefix = "image_"
    image_ext = ".jpg"

    # Find the next available image number
    img_count = 0
    count = 0
    while True:
        image_name = f"{image_name_prefix}{count:03d}{image_ext}"
        image_path = os.path.join(images_folder, image_name)
        if not os.path.exists(image_path):
            break
        count += 1
    img_count = count
    print(img_count)
    # Set camera control variables
    with picam2.controls as controls:
        controls.ColourGains = (REDVAL, BLUEVAL)
        controls.AnalogueGain = AGAINVAL
        controls.Contrast = CONTRAST
        controls.Brightness = BRIGHT

    # Turn LED on
    GPIO.output(LED_PIN1, GPIO.HIGH)
    GPIO.output(LED_PIN2, GPIO.HIGH)
    GPIO.output(LED_PIN3, GPIO.HIGH)
    time.sleep(0.5)
    # Turn LED off
    GPIO.output(LED_PIN1, GPIO.LOW)
    GPIO.output(LED_PIN2, GPIO.LOW)
    GPIO.output(LED_PIN3, GPIO.LOW)
    time.sleep(0.5)

    while True:
        if cv2.waitKey(1) == ord("q"):
            break
        if GPIO.input(BUTTON_01) == GPIO.LOW:
            # Toggle photo capture flag
            time.sleep(1)
            if img_count > 0:
                img_count = img_count + 10
            CAMERA_CAPTURE = not CAMERA_CAPTURE

            if CAMERA_CAPTURE:
                # Turn LED on
                GPIO.output(LED_PIN1, GPIO.HIGH)

                while CAMERA_CAPTURE:
                    # Capture photo
                    im = picam2.capture_array()

                    # Create the filename for the new image
                    filename = f"{image_name_prefix}{img_count:03d}{image_ext}"
                    img_count += 1
                    im_output_path = os.path.join(images_folder, filename)

                    # Save the image and print a message
                    cv2.imwrite(im_output_path, im)
                    print(f"Captured {filename}")

                    if GPIO.input(BUTTON_01) == GPIO.LOW:
                        # Toggle photo capture flag
                        time.sleep(1)
                        CAMERA_CAPTURE = not CAMERA_CAPTURE
                    # Wait for the capture interval
                    time.sleep(CAPTURE_INTERVAL)

                # Turn LED off
                GPIO.output(LED_PIN1, GPIO.LOW)

        time.sleep(0.1)  # Small delay to prevent button bounce
    GPIO.cleanup()
    picam2.stop()
