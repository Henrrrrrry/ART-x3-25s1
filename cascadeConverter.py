"""
Full Body Detection System using OpenMV Camera

Authors:
    Author 1: [Zijun Zhou]: setup basic converter, fix bugs
    Author 2: [Hongyu Li]: setup basic converter

Description:
This system uses an OpenMV camera to detect full bodies using Haar cascades,
identifies the largest detected body, and provides visual feedback with bounding boxes
and tracking lines.

Hardware Requirements:
- OpenMV Camera (H7/M7)
- Built-in LED for status indication

Input:
- Camera feed from OpenMV sensor
- Haar cascade file: "haarcascade_fullbody.cascade"

Output:
- Real-time detection display on camera
- Visual bounding boxes around detected bodies
- Tracking line from image center to largest body center
- LED status indication (on when body detected, off when no detection)
- Serial console output with detection coordinates and FPS

Functions:
- Main detection loop with real-time processing
- Largest object selection algorithm
- Visual feedback rendering
"""

import image
import time
import pyb
import sensor

# ============================================================================
# HARDWARE INITIALIZATION
# ============================================================================
# Initialize status LED (LED 3 - blue LED on OpenMV)
led = pyb.LED(3)

# ============================================================================
# CAMERA CONFIGURATION
# ============================================================================
# Reset and configure camera sensor
sensor.reset()                          # Reset camera sensor
sensor.set_contrast(3)                  # Set contrast level for better detection
sensor.set_gainceiling(16)              # Set maximum gain ceiling
sensor.set_framesize(sensor.QVGA)       # Set resolution to 320x240 (QVGA)
sensor.set_pixformat(sensor.GRAYSCALE)  # Use grayscale for faster processing

# ============================================================================
# IMAGE PARAMETERS
# ============================================================================
# Get camera image dimensions and calculate center point
WIDTH = sensor.width()                  # Image width (320 pixels)
HEIGHT = sensor.height()                # Image height (240 pixels)
CENTER_X = int(WIDTH / 2 + 0.5)         # Center X coordinate (160)
CENTER_Y = int(HEIGHT / 2 + 0.5)        # Center Y coordinate (120)

# ============================================================================
# PERFORMANCE MONITORING
# ============================================================================
# Initialize clock for FPS calculation
clock = time.clock()

# ============================================================================
# OBJECT DETECTION SETUP
# ============================================================================
# Load Haar cascade classifiers for detection
# Note: frontalface cascade is built-in, fullbody cascade is external file
face_cascade = image.HaarCascade("frontalface", stages=25)
fullbody_cascade = image.HaarCascade("haarcascade_fullbody.cascade", stages=25)

# ============================================================================
# MAIN DETECTION LOOP
# ============================================================================
while(True):
    # ========================================================================
    # FRAME CAPTURE AND TIMING
    # ========================================================================
    # Take timestamp for FPS calculation
    clock.tick()

    # Capture current frame from camera
    img = sensor.snapshot()

    # ========================================================================
    # OBJECT DETECTION
    # ========================================================================
    # Detect full body objects in current frame
    # threshold=0.75: Detection confidence threshold (higher = more strict)
    # scale_factor=1.2: Multi-scale detection parameter
    objects = img.find_features(fullbody_cascade, threshold=0.75, scale_factor=1.2)

    # ========================================================================
    # LARGEST OBJECT SELECTION
    # ========================================================================
    # Initialize variables for finding largest detected body
    largest_face_size = 0      # Size of largest detected object
    largest_face_bb = None     # Bounding box of largest object

    # Process all detected objects
    for r in objects:
        # Calculate object size (area of bounding box)
        face_size = r[2] * r[3]  # width * height

        # Check if this is the largest object found so far
        if (face_size > largest_face_size):
            largest_face_size = face_size
            largest_face_bb = r

        # Draw bounding rectangle around all detected objects
        # r format: (x, y, width, height)
        img.draw_rectangle(r)

    # ========================================================================
    # TRACKING AND VISUAL FEEDBACK
    # ========================================================================
    # Process largest detected object if any object was found
    if largest_face_bb is not None:
        # ====================================================================
        # STATUS INDICATION
        # ====================================================================
        # Turn on status LED to indicate successful detection
        led.on()

        # ====================================================================
        # CONSOLE OUTPUT
        # ====================================================================
        # Print detection information to serial console
        print("Body detected:", largest_face_bb)

        # ====================================================================
        # CALCULATE OBJECT CENTER
        # ====================================================================
        # Find center coordinates of largest detected body
        # largest_face_bb format: (x, y, width, height)
        face_x = largest_face_bb[0] + int((largest_face_bb[2])/2 + 0.5)  # Center X
        face_y = largest_face_bb[1] + int((largest_face_bb[3])/2 + 0.5)  # Center Y

        # ====================================================================
        # VISUAL TRACKING LINE
        # ====================================================================
        # Draw tracking line from image center to detected body center
        # This helps visualize the offset between camera center and target
        img.draw_line(CENTER_X, CENTER_Y, face_x, face_y)

    else:
        # ====================================================================
        # NO DETECTION HANDLING
        # ====================================================================
        # Turn off status LED when no body is detected
        led.off()

    # ========================================================================
    # PERFORMANCE MONITORING OUTPUT
    # ========================================================================
    # Print current frame rate to console
    print("FPS:", clock.fps())