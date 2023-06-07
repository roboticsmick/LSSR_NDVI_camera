import cv2
import os
import time
import math
import serial
from picamera2 import Picamera2
import numpy as np
from fastiecm import fastiecm
from pathlib import Path
import RPi.GPIO as GPIO
from ultralytics import YOLO

# Pin definitions
BUTTON_01 = 4
LED_PIN1 = 17
LED_PIN2 = 27
LED_PIN3 = 22
LED_PIN4 = 10
LED_PIN5 = 24
LED_PIN6 = 23

# BGR Colour palette
BLUE = (239, 107, 23)  # Blue for class 0 = path
RED = (48, 62, 255)  # Red for class 1 = weeds
YELLOW = (41, 181, 247)  # Yellow for direction
GREEN = (82, 156, 23)  # Green
WHITE = (255, 255, 255)  # White
CYAN = (199, 199, 80)  # White
ORANGE = [0, 153, 255]  # Orange
COLOUR = [BLUE, GREEN, YELLOW, RED, WHITE]

# Class labels
PATH = 0
WEED = 1
HEADING = 2
OBJECTS = [PATH, WEED]
OBJECT_LABELS = ["PATH", "WEED"]

# Global variables
global TARGET
global DIRECTION
global STATUS
global ROBOT_STATUS

# Initial values
WEEDS = 0
ANGLE = 90
WATCHDOG = 0
ROBOT_STATUS = False

# Threshold values
CONF_THRESHOLD_PATH = 70
CONF_THRESHOLD_WEED = 50

# Camera settings
REDVAL = 0.544
BLUEVAL = 1.248
AGAINVAL = 9.286
CONTRAST = 1.376
BRIGHT = 0.0

# Camera resolution
IMG_X = 640
IMG_Y = 360
IMG_X_CENTRE = IMG_X // 2

# Set the spray threshold to target
WEED_CAMERA_Y_SPRAY_LIMIT = 40
WEED_Y_AXIS_THRESHOLD = IMG_Y - WEED_CAMERA_Y_SPRAY_LIMIT
WEED_X_SPRAY_LIMIT = 120
SPRAY_LEFT_THRESHOLD = IMG_X_CENTRE - WEED_X_SPRAY_LIMIT
SPRAY_RIGHT_THRESHOLD = IMG_X_CENTRE + WEED_X_SPRAY_LIMIT

# Font settings
FONT_TRANS_X = 2
FONT_TRANS_Y = 12
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_D = cv2.FONT_HERSHEY_DUPLEX
FONT_SMALL = 0.4
FONT_LARGE = 0.5
FONT_THICKNESS = 1
BACKGROUND_LABEL_WIDTH = 68
BACKGROUND_LABEL_HEIGHT = 10
BOUNDARY_MAX_Y_HEIGHT = 50

# Initialise YOLO Model
model = YOLO("weed_detection_model.pt")

# Serial port for Arduino
arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

# File paths
base_folder = Path(__file__).parent.resolve()

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


# Classify and label the objects detect by model
def object_classification(results, im):
    global WEEDS
    global ANGLE
    global WATCHDOG

    WATCHDOG = 0

    for result in results:
        paths = []  # List to store path boundary coordinates
        weeds = []  # List to store weed coordinates
        object_data = result.boxes  # Boxes object for bbox outputs

        # Iterate over each box in the results
        for obj in object_data:
            # Get coordinates of the object
            x1, y1, x2, y2 = [int(coord) for coord in obj.xyxy[0].tolist()]
            # Get confidence score
            confidence = int(obj.conf[0].tolist() * 100)
            # Get class ID
            object_id = int(obj.cls.tolist()[0])
            # Check if the box represents a path (class 0)
            if object_id == PATH:
                # Draw a rectangle around the path
                im = draw_boundary_box(im, x1, y1, x2, y2, object_id, confidence)

                # Store path coordinates if confidence is above threshold
                if confidence >= CONF_THRESHOLD_PATH:
                    paths.append((x1, y1, x2, y2, confidence))
            else:
                # Draw a rectangle around the weed
                im = draw_boundary_box(im, x1, y1, x2, y2, object_id, confidence)

                # Check if weed coordinates are within the spray threshold to target
                proximity_check = weed_proximity(x1, x2, y2)

                # If X coordinates are valid, store weed coordinates and draw 'X' marker
                if proximity_check is not None:
                    # Store weed coordinates that are close to the bottom of the image
                    weeds.append((x1, x2, proximity_check[0], proximity_check[1]))
        if paths:
            # Set WATCHDOG to 1 indicating path detected to pass to Arduino
            WATCHDOG = 1
            # Find the closest path to the robot
            closest_path = find_closest_path(paths)
            # Draw the direction of the robot on the image
            im, direction = draw_robot_direction(im, closest_path)
            # Set the angle of the robot to pass to the Arduino
            ANGLE = direction
            angle_string = str(ANGLE)
            draw_heading(im, angle_string)
            # Get sprayer targets based on weed coordinates near robot
            if weeds:
                # Calculate the targets to spray to pass to the Arduino
                WEEDS = get_weed_segments(im, weeds, closest_path)
                # Draw the sprayer control on the image
            else:
                # If there are no weeds, set weed targets to "000" to represent no targets
                WEEDS = 0
        else:
            # Do no update angle unless path detected
            angle_string = "NOT DETECTED"
            draw_heading(im, angle_string)
            # Set watchdog to 0 indicating no path is present to pass to Arduino
            WATCHDOG = 2
            # If path not detected set weed targets to not spray to represent no confirmed targets
            WEEDS = 0
        draw_sprayer_control(im, WEEDS)

    return im, WEEDS, ANGLE, WATCHDOG


# Calculate the direction of the robot relative to the closest path
def draw_robot_direction(im, closest_path):
    x1, y1, x2, y2, confidence = closest_path
    # Calculate the centroid x-coordinate of the closest path
    centroid_x = (x1 + x2) // 2

    # Set the centroid y-coordinate as the bottom y-coordinate of the path
    centroid_y = y2

    # Calculate the x-coordinate of the bottom center of the image
    bottom_center_x = IMG_X // 2

    # Set the y-coordinate of the bottom center of the image
    bottom_center_y = IMG_Y

    # Draw a cross marker at the centroid of the closest path
    cv2.drawMarker(
        im,
        (centroid_x, centroid_y),
        COLOUR[PATH],
        markerType=cv2.MARKER_CROSS,
        thickness=3,
    )

    # Determine the direction angle relative to the closest path
    path_coordinates = calc_path_direction(closest_path)

    # Extract the angle obtained from the angle_to_path function
    angle_to_path = int(path_coordinates[2])

    # Draw a line from the bottom center of the image towards the direction angle
    cv2.line(
        im,
        (bottom_center_x, bottom_center_y),
        (path_coordinates[0], path_coordinates[1]),
        BLUE,
        thickness=2,
    )
    return im, angle_to_path


# Find the closest path object
def find_closest_path(paths):
    # Sort paths by y-coordinate (ascending order)
    paths.sort(key=lambda path: path[1])
    # Return the path with the lowest y-coordinate
    return paths[0]


# Calculate angle to path centre
def calc_path_direction(path_coords):
    # Extract coordinates of the closest path
    x1, y1, x2, y2, confidence = path_coords
    # Calculate the x-coordinate of the bottom center of the image
    bottom_center_x = IMG_X // 2
    # Set the y-coordinate of the bottom center of the image
    bottom_center_y = IMG_Y
    # Calculate the x-coordinate of the center of the path
    path_center_x = (x1 + x2) // 2
    # Calculate the y-coordinate of the center of the path
    path_center_y = IMG_Y // 2
    # Calculate the horizontal distance from the bottom center of the image to the path center
    dx = path_center_x - bottom_center_x
    # Calculate the vertical distance from the bottom center of the image to the path center
    dy = IMG_Y // 2
    # Calculate the angle in radians from the horizontal axis to the path center
    angle_rad = np.arctan2(dy, dx)
    # Convert the angle to degrees
    angle_deg = int(np.rad2deg(angle_rad))
    # Constrain the angle to be between 30 and 150 degrees
    constrained_angle = min(150, max(30, angle_deg))
    # Store the path center coordinates and the adjusted angle
    direction = (path_center_x, path_center_y, constrained_angle)
    return direction


# Identify which segments the weeds to set spray targets
def get_weed_segments(im, weeds, path):
    # Calculate the width of the path boundary
    spray_width = SPRAY_RIGHT_THRESHOLD - SPRAY_LEFT_THRESHOLD

    # Divide the path boundary into three equal segments
    spray_segment = spray_width // 5

    # Compute the start and end x-coordinates of each segment
    spray_segment_starts = [SPRAY_LEFT_THRESHOLD + i * spray_segment for i in range(5)]
    spray_segment_ends = [start + spray_segment for start in spray_segment_starts]

    # Initialize the binary representation
    segments = [0, 0, 0, 0, 0]  # 0 = no weed, 1 = weed

    for weed_coords in weeds:
        # Check which segments the weed overlaps with
        x_left = weed_coords[0]
        x_right = weed_coords[1]
        x_centre = weed_coords[2]

        if weed_coords[2] >= path[0] and weed_coords[2] <= path[2]:
            # Draw a cross marker at the center of the weed
            cv2.drawMarker(
                im,
                (weed_coords[2], weed_coords[3]),
                COLOUR[WEED],
                markerType=cv2.MARKER_CROSS,
                thickness=3,
            )

        # Check if the left edge of the weed is within any segment
        for i, segment_start in enumerate(spray_segment_starts):
            segment_end = spray_segment_ends[i]
            if x_left >= segment_start and x_left < segment_end:
                segments[i] = 1  # Overlaps with the segment
                break  # No need to check further segments

        # Check if the right edge of the weed is within any segment
        for i, segment_start in enumerate(spray_segment_starts):
            segment_end = spray_segment_ends[i]
            if x_right >= segment_start and x_right < segment_end:
                segments[i] = 1  # Overlaps with the segment
                break  # No need to check further segments

        # Check if the center of the weed is within any segment
        for i, segment_start in enumerate(spray_segment_starts):
            segment_end = spray_segment_ends[i]
            if x_centre >= segment_start and x_centre < segment_end:
                segments[i] = 1  # Overlaps with the segment
                break  # No need to check further segments

    # Output the updated list as a 3-digit binary string
    binary = int("".join(str(seg) for seg in segments), 2)
    return binary


# Draw heading angle to image
def draw_heading(im, angle):
    cv2.rectangle(
        img,
        (0, 0),
        (0 + 220, 0 + 20),
        WHITE,  # white color
        cv2.FILLED,
    )
    # Draw the heading angle
    cv2.putText(
        im,
        "HEADING: " + angle,
        (10, 16),
        FONT_D,
        FONT_LARGE,
        COLOUR[PATH],
        FONT_THICKNESS,
    )
    return im


# Draw sprayer control settings to image
def draw_sprayer_control(im, weeds):
    # Calculate the width of the path boundary
    spray_width = SPRAY_RIGHT_THRESHOLD - SPRAY_LEFT_THRESHOLD
    # Divide the path boundary into three equal segments
    spray_segment = spray_width // 4
    # Draw V at each segment
    for i in range(5):
        x_segment = SPRAY_LEFT_THRESHOLD + (i * spray_segment)
        cv2.putText(
            im,
            "V",
            (x_segment, WEED_Y_AXIS_THRESHOLD),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.4,
            ORANGE,
            2,
        )

    cv2.rectangle(
        img,
        (0, 22),
        (0 + 220, 22 + 20),
        WHITE,  # white color
        cv2.FILLED,
    )
    # Draw the sprayer control settings
    weedstring = ""
    for i in range(4, -1, -1):  # Reverse loop iteration
        if ((weeds >> i) & 1) == 1:
            weedstring += "X"
        else:
            weedstring += "O"
        # Add a dash between the bits, except for the last bit
        if i > 0:
            weedstring += "-"
    # Draw the sprayer control settings
    cv2.putText(
        im,
        "SPRAYING: " + weedstring,
        (10, 36),
        FONT_D,
        FONT_LARGE,
        ORANGE,
        FONT_THICKNESS,
    )

    return im


# Check weed is close to spray to set target
def weed_proximity(x1, x2, y2):
    # Calculate the x-coordinate of the centroid of the bounding box
    centroid_x = (x1 + x2) // 2

    # Set the y-coordinate of the centroid of the bounding box as the bottom y-coordinate
    centroid_y = y2

    # Check if the y-coordinate of the centroid is greater than or equal to a threshold value
    if centroid_y >= WEED_Y_AXIS_THRESHOLD:
        X = (centroid_x, centroid_y)
        # Return the 'X' marker coordinates
        return X
    else:
        # If the y-coordinate does not meet the threshold, return None
        return None


# Draw boundary box and confidence label for object
def draw_boundary_box(im, x1, y1, x2, y2, object_id, confidence):
    # Prevents boundary box overlapping with data display
    if y1 < BOUNDARY_MAX_Y_HEIGHT:
        y1 = BOUNDARY_MAX_Y_HEIGHT

    # Draw a rectangle around the box (weed)
    cv2.rectangle(im, (x1, y1), (x2, y2), COLOUR[object_id], thickness=2)
    # Create
    label = OBJECT_LABELS[object_id] + " " + str(confidence) + "%"
    # Calculate the coordinates for the white box background
    box_x = x1 + FONT_TRANS_X
    box_y = y1 + FONT_TRANS_Y - BACKGROUND_LABEL_HEIGHT
    # Draw the white background box
    cv2.rectangle(
        img,
        (box_x, box_y),
        (box_x + BACKGROUND_LABEL_WIDTH, box_y + BACKGROUND_LABEL_HEIGHT),
        COLOUR[4],  # white color
        cv2.FILLED,
    )
    # Draw label
    cv2.putText(
        img,
        label,
        (x1 + FONT_TRANS_X, y1 + FONT_TRANS_Y),
        FONT,
        FONT_SMALL,
        COLOUR[object_id],
        FONT_THICKNESS,
    )
    return im


# Stop robot and turn on LED lights
def stopped_robot():
    WEEDS = 0
    ANGLE = 90
    WATCHDOG = 0
    # Message arduino
    message_arduino(WEEDS, ANGLE, WATCHDOG)
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


# Activate robot LED lights
def robot_activate():
    # Cycle LED on and off
    GPIO.output(LED_PIN1, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(LED_PIN1, GPIO.LOW)
    GPIO.output(LED_PIN2, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(LED_PIN2, GPIO.LOW)
    GPIO.output(LED_PIN3, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(LED_PIN3, GPIO.LOW)


def loop_through_images(folder_path):
    image_extensions = [".jpg", ".jpeg", ".png"]  # Add more extensions if needed
    image_files = [
        f
        for f in os.listdir(folder_path)
        if os.path.isfile(os.path.join(folder_path, f))
        and any(f.lower().endswith(ext) for ext in image_extensions)
    ]
    image_files.sort()  # Sort the image files by name

    for image_file in image_files:
        image_path = os.path.join(folder_path, image_file)
        yield image_path


def message_arduino(weeds, watchdog, angle):
    data = (
        weeds.to_bytes(1, "little")
        + watchdog.to_bytes(1, "little")
        + angle.to_bytes(2, "little")
    )
    arduino.write(data)


if __name__ == "__main__":
    # Initialise the images folder for testing
    # Get the current directory
    current_directory = os.getcwd()
    images_folder = "images"
    images_demo = "demo_03"
    image_folder_path = os.path.join(current_directory, images_folder, images_demo)
    image_generator = loop_through_images(image_folder_path)

    # Initialise the robot
    robot_activate()
    # Initialise camera
    picam2 = Picamera2()
    picam2.set_controls(
        {"ExposureTime": 12000, "AnalogueGain": 1.2, "AwbEnable": 0, "AeEnable": 0}
    )
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.main.size = (640, 360)
    picam2.preview_configuration.controls.FrameRate = 15
    picam2.preview_configuration.align()
    picam2.start()
    time.sleep(2)
    with picam2.controls as controls:
        controls.ColourGains = (REDVAL, BLUEVAL)
        controls.AnalogueGain = AGAINVAL
        controls.Contrast = CONTRAST
        controls.Brightness = BRIGHT

    while True:
        try:
            # Get the next image file from the image generator
            image_file = next(image_generator)
            img = cv2.imread(image_file)
            # Get image from Raspberry Pi camera
            # img = picam2.capture_array()
            # Use the model to predict based on the image
            results = model.predict(source=img)
            # Perform object classification on the image results
            img_model, WEEDS, ANGLE, WATCHDOG = object_classification(results, img)
            # Send the object classification results to Arduino
            message_arduino(WEEDS, ANGLE, WATCHDOG)
            # Print the Arduino commands - DELETE WHEN LIVE
            # Display the original image
            cv2.imshow("Original", img_model)
            # Wait for the 'q' key to be pressed
            if cv2.waitKey(1) == ord("q"):
                break
            # Check if BUTTON is pressed stop robot
            if GPIO.input(BUTTON_01) == GPIO.LOW:
                # Set ROBOT_STATUS to False to stop the robot
                ROBOT_STATUS = False
                # Sleep to minimise bounce
                time.sleep(0.5)
                # Enter a loop while ROBOT_STATUS is False
                while ROBOT_STATUS is False:
                    # Stop the robot
                    stopped_robot()
                    # Check if BUTTON_01 is pressed again
                    if GPIO.input(BUTTON_01) == GPIO.LOW:
                        # Set ROBOT_STATUS to True to activate the robot
                        ROBOT_STATUS = True
                        # Activate the robot
                        robot_activate()
                        # Sleep to minimise bounce
                        time.sleep(0.5)
        # If there are no more images to process, break the loop
        except StopIteration:
            print("All images have been processed.")
            break
    # Clean up
    cv2.destroyAllWindows()
    GPIO.cleanup()
    picam2.stop()
