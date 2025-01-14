# YOLOV8 Autonomous ROBOGOAT

Overview
ROBOGOAT is an autonomous weeding robot developed as a final year mechatronics project at Queensland University of Technology. The robot is designed to navigate public footpaths while detecting and eliminating weeds using computer vision and targeted pesticide application.
Key Features

Custom trained YOLOv8 object detection model for weed and path detection
IR-filtered computer vision system using Raspberry Pi Global Shutter camera
Targeted pesticide delivery system to minimize chemical usage
Autonomous navigation using AI path detection
Custom PCB design for power management and camera control
Solar-powered operation for extended runtime

Technical Stack

Vision System: Python, OpenCV, YOLOv8
Hardware: Raspberry Pi 4, Arduino Mega, Custom PCB
Computer Vision: NDVI (Normalized Difference Vegetation Index), IR filtering
Control: C++ for motor and sprayer control
Design: Fusion360 for chassis and components

Results

Path detection accuracy: >90%
Weed detection accuracy: 49% overall, higher for weeds directly on paths
Successfully implemented safety features including watchdog states
Demonstrated reliable autonomous operation during field testing

Project Purpose
The project aims to address the challenges of footpath maintenance in urban areas, specifically:

Reducing manual labor costs in weed management
Minimizing human exposure to pesticides
Preventing footpath damage from weed growth
Reducing overall pesticide usage through targeted application
