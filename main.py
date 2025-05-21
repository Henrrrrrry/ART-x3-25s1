"""
Upper Body Tracking System using OpenMV Camera and Servo Control

Authors: 
    Author 1: [Qianwen Shen]: motor rotation algorithm
    Author 2: [Hongyu Li]: camera tracking algorithm, integration of two features
    Author 3: [Zijun Zhou]: camera tracking algorithm
    Author 4: [Leliang Wang]: motor rotation algorithm

Description:
This system uses an OpenMV camera to detect and track upper bodies using Haar cascades,
then controls two motors via PCA9685 to follow the detected target.

Hardware Requirements:
- OpenMV Camera (H7/M7)
- PCA9685 PWM Driver Board
- 2x SG90R Motors (for horizontal and vertical movement)
- I2C connections between OpenMV and PCA9685
- 5V external Power for PCA9685

Input:
- Camera feed from OpenMV sensor
- Haar cascade file: "haarcascade_upperbody.cascade"
- I2C communication with PCA9685

Output:
- Real-time tracking display on camera
- Servo motor control signals via PCA9685
- PWM signals to servo motors for pan/tilt movement

Functions:
- reset_pca9685(): Initialize PCA9685 PWM driver
- set_servo_pulse(): Send PWM signals to specific servo channel
- calculate_iou(): Calculate Intersection over Union for object tracking
- force_stop_motors(): Emergency stop for all servo motors
"""

import sensor, image, time
import gc
from machine import I2C

# Enable memory management for stable operation
gc.enable()
gc.collect()

# ============================================================================
# CAMERA INITIALIZATION
# ============================================================================
# Initialize camera with grayscale mode for better processing speed
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # Grayscale for faster processing
sensor.set_framesize(sensor.QVGA)       # 320x240 resolution
sensor.skip_frames(time=2000)           # Wait for camera to stabilize
sensor.set_auto_gain(False)             # Disable auto gain for consistent exposure
sensor.set_auto_whitebal(False)         # Disable auto white balance
sensor.set_framerate(20)                # Set frame rate to 20 FPS

# Get image dimensions and calculate center point
WIDTH = sensor.width()      # Image width (320 pixels)
HEIGHT = sensor.height()    # Image height (240 pixels)
CENTER_X = WIDTH // 2       # Center X coordinate (160)
CENTER_Y = HEIGHT // 2      # Center Y coordinate (120)

# ============================================================================
# PCA9685 PWM DRIVER CONFIGURATION
# ============================================================================
# PCA9685 register addresses for I2C communication
PCA9685_ADDR = 0x40    # I2C address of PCA9685
MODE1 = 0x00           # Mode register 1
PRESCALE = 0xFE        # Prescaler register for PWM frequency
LED0_ON_L = 0x06       # First PWM channel register

# Create I2C object for communication with PCA9685
# I2C(2) uses pins P4 (SDA) and P5 (SCL) on OpenMV
i2c = I2C(2, freq=100000)  # 100kHz I2C frequency

# ============================================================================
# PCA9685 INITIALIZATION AND CONTROL FUNCTIONS
# ============================================================================
def reset_pca9685():
    """
    Initialize PCA9685 PWM driver for servo control
    
    Input: None
    Output: Boolean - True if initialization successful, False if failed
    """
    try:
        # Read current mode1 register
        mode1 = i2c.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
        # Put PCA9685 to sleep for configuration
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 | 0x10]))
        time.sleep(0.005)

        # Calculate prescaler for 50Hz PWM frequency (standard for servos)
        prescale = 25000000 // (4096 * 50) - 1
        i2c.writeto_mem(PCA9685_ADDR, PRESCALE, bytes([prescale]))
        time.sleep(0.005)

        # Wake up PCA9685
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10]))
        time.sleep(0.005)

        # Enable auto-increment and restart
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10 | 0xa0]))
        time.sleep(0.005)

        return True
    except Exception as e:
        return False

def set_servo_pulse(channel, pulse_us):
    """
    Send PWM pulse to specific servo channel
    
    Input: 
        channel (int) - Servo channel number (0-15)
        pulse_us (int) - Pulse width in microseconds (1000-2000)
    Output: Boolean - True if successful, False if failed
    """
    try:
        # Convert microseconds to 12-bit value (0-4095)
        pulse_count = pulse_us * 4096 // 20000
        channel_base = LED0_ON_L + (channel * 4)

        # Write PWM values to PCA9685 registers
        i2c.writeto_mem(PCA9685_ADDR, channel_base, bytes([0]))
        time.sleep_ms(1)
        i2c.writeto_mem(PCA9685_ADDR, channel_base + 1, bytes([0]))
        time.sleep_ms(1)
        i2c.writeto_mem(PCA9685_ADDR, channel_base + 2, bytes([pulse_count & 0xFF]))
        time.sleep_ms(1)
        i2c.writeto_mem(PCA9685_ADDR, channel_base + 3, bytes([(pulse_count >> 8) & 0x0F]))
        time.sleep_ms(1)

        return True
    except Exception as e:
        return False

# Initialize PCA9685 PWM driver
reset_status = reset_pca9685()

# ============================================================================
# SERVO CONTROL PARAMETERS
# ============================================================================
# Servo pulse width values in microseconds
STOP_PULSE = 1520      # Neutral position (no movement)
FORWARD_PULSE = 1530   # Fast forward movement
REVERSE_PULSE = 1455   # Fast reverse movement  
SLOW_FORWARD = 1525    # Slow forward movement
SLOW_REVERSE = 1475    # Slow reverse movement

# Servo channel assignments on PCA9685
H_CHANNEL = 0  # Horizontal servo channel
V_CHANNEL = 1  # Vertical servo channel

# ============================================================================
# OBJECT DETECTION SETUP
# ============================================================================
# Load Haar cascade for upper body detection
upperbody_cascade = image.HaarCascade("haarcascade_upperbody.cascade", stages=17)

# ============================================================================
# TRACKING CONTROL PARAMETERS
# ============================================================================
# Error thresholds for servo movement decisions
SMALL_ERROR = 15   # Minimum error to trigger slow movement
LARGE_ERROR = 40   # Error threshold for fast movement

# Tracking loss management
MAX_LOST_FRAMES = 3    # Frames to wait before considering target lost
FORCE_STOP_FRAMES = 5  # Frames to ensure complete motor stop
force_stop_counter = 0

# ============================================================================
# INITIAL SERVO POSITIONING
# ============================================================================
# Set both servos to neutral position at startup
set_servo_pulse(H_CHANNEL, STOP_PULSE)
time.sleep_ms(100)
set_servo_pulse(V_CHANNEL, STOP_PULSE)
time.sleep(1)

# ============================================================================
# TRACKING STATE VARIABLES
# ============================================================================
# Target tracking variables
last_tracked_pos = None    # Last known position of tracked object
track_lost_count = 0       # Counter for consecutive frames with no detection

# Motor status variables
last_h_pulse = STOP_PULSE  # Last horizontal servo pulse value
last_v_pulse = STOP_PULSE  # Last vertical servo pulse value

# Motor movement tracking
motor_moving = False           # Flag indicating if motors are currently moving
last_detection_time = time.ticks_ms()  # Timestamp of last successful detection

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
def calculate_iou(boxA, boxB):
    """
    Calculate Intersection over Union (IoU) between two bounding boxes
    Used for object tracking consistency between frames
    
    Input:
        boxA (tuple) - First bounding box (x, y, width, height)
        boxB (tuple) - Second bounding box (x, y, width, height)
    Output:
        float - IoU value between 0 and 1 (higher = more overlap)
    """
    # Calculate intersection coordinates
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
    yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])

    # Calculate intersection area
    interArea = max(0, xB - xA) * max(0, yB - yA)

    # Calculate individual box areas
    boxAArea = boxA[2] * boxA[3]
    boxBArea = boxB[2] * boxB[3]

    # Calculate IoU
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

def force_stop_motors():
    """
    Emergency stop function for all servo motors
    Sends multiple stop commands to ensure motors halt
    
    Input: None
    Output: None (modifies global motor state variables)
    """
    global force_stop_counter, motor_moving, last_h_pulse, last_v_pulse

    # Send multiple stop commands for reliability
    for i in range(3):
        set_servo_pulse(H_CHANNEL, STOP_PULSE)
        time.sleep_ms(10)
        set_servo_pulse(V_CHANNEL, STOP_PULSE)
        time.sleep_ms(10)

    # Reset motor state variables
    last_h_pulse = STOP_PULSE
    last_v_pulse = STOP_PULSE
    motor_moving = False
    force_stop_counter = 0

# ============================================================================
# MAIN TRACKING LOOP
# ============================================================================
# Initialize frame rate clock
clock = time.clock()

while(True):
    clock.tick()

    # ========================================================================
    # IMAGE CAPTURE AND PROCESSING
    # ========================================================================
    # Capture current frame from camera
    img = sensor.snapshot()

    # Detect upper body objects in current frame
    # threshold=0.70: Detection confidence threshold
    # scale_factor=1.2: Multi-scale detection parameter
    upperbody_objects = img.find_features(upperbody_cascade, threshold=0.70, scale_factor=1.2)

    # Initialize tracking variables for current frame
    tracked_object = None
    current_time = time.ticks_ms()

    # ========================================================================
    # OBJECT TRACKING LOGIC
    # ========================================================================
    # Case 1: Currently tracking a target
    if last_tracked_pos:
        best_match = None
        best_score = 0

        # Find best matching object using IoU
        for obj in upperbody_objects:
            iou = calculate_iou(last_tracked_pos, obj)
            if iou > best_score and iou > 0.3:  # Minimum 30% overlap required
                best_score = iou
                best_match = obj

        if best_match:
            # Successfully matched previous target
            tracked_object = best_match
            track_lost_count = 0
            last_detection_time = current_time
        else:
            # Target lost, increment counter
            track_lost_count += 1

            # Stop tracking if target lost for too many frames
            if track_lost_count > MAX_LOST_FRAMES:
                last_tracked_pos = None
                force_stop_motors()

    # Case 2: Not currently tracking a target
    if not last_tracked_pos:
        if upperbody_objects:
            # Select largest detected object as new target
            tracked_object = max(upperbody_objects, key=lambda r: r[2] * r[3])
            track_lost_count = 0
            last_detection_time = current_time
        else:
            # No objects detected, check if motors should stop
            time_since_detection = time.ticks_diff(current_time, last_detection_time)
            if time_since_detection > 500:  # 500ms timeout
                if motor_moving:
                    force_stop_motors()

    # ========================================================================
    # SERVO CONTROL LOGIC
    # ========================================================================
    if tracked_object:
        # Apply smoothing to reduce jitter
        if last_tracked_pos:
            alpha = 0.7  # Smoothing factor (0.7 = 70% new, 30% old)
            x = int(alpha * tracked_object[0] + (1 - alpha) * last_tracked_pos[0])
            y = int(alpha * tracked_object[1] + (1 - alpha) * last_tracked_pos[1])
            w = int(alpha * tracked_object[2] + (1 - alpha) * last_tracked_pos[2])
            h = int(alpha * tracked_object[3] + (1 - alpha) * last_tracked_pos[3])
            tracked_object = (x, y, w, h)

        # Update last known position
        last_tracked_pos = tracked_object

        # ====================================================================
        # VISUAL FEEDBACK
        # ====================================================================
        # Draw tracking rectangle and center cross on image
        img.draw_rectangle(tracked_object, color=(255, 0, 0))  # Red rectangle
        center_x = tracked_object[0] + tracked_object[2] // 2
        center_y = tracked_object[1] + tracked_object[3] // 2
        img.draw_cross(center_x, center_y, color=(255, 0, 0))  # Red cross

        # ====================================================================
        # ERROR CALCULATION AND MOVEMENT DECISION
        # ====================================================================
        # Calculate tracking errors (distance from image center)
        x_error = CENTER_X - center_x  # Horizontal error
        y_error = CENTER_Y - center_y  # Vertical error

        # Determine if movement is needed
        should_move = abs(x_error) > SMALL_ERROR or abs(y_error) > SMALL_ERROR

        if should_move:
            motor_moving = True
            force_stop_counter = 0

            try:
                # ============================================================
                # HORIZONTAL SERVO CONTROL
                # ============================================================
                h_pulse = STOP_PULSE
                if abs(x_error) > SMALL_ERROR:
                    if x_error > 0:  # Target is to the left, move camera right
                        h_pulse = FORWARD_PULSE if abs(x_error) > LARGE_ERROR else SLOW_FORWARD
                    else:  # Target is to the right, move camera left
                        h_pulse = REVERSE_PULSE if abs(x_error) > LARGE_ERROR else SLOW_REVERSE

                # Only send command if pulse value changed
                if h_pulse != last_h_pulse:
                    last_h_pulse = h_pulse
                    set_servo_pulse(H_CHANNEL, h_pulse)
                    time.sleep_ms(20)

                # ============================================================
                # VERTICAL SERVO CONTROL
                # ============================================================
                v_pulse = STOP_PULSE
                if abs(y_error) > SMALL_ERROR:
                    if y_error > 0:  # Target is above, move camera up
                        v_pulse = REVERSE_PULSE if abs(y_error) > LARGE_ERROR else SLOW_REVERSE
                    else:  # Target is below, move camera down
                        v_pulse = FORWARD_PULSE if abs(y_error) > LARGE_ERROR else SLOW_FORWARD

                # Only send command if pulse value changed
                if v_pulse != last_v_pulse:
                    last_v_pulse = v_pulse
                    set_servo_pulse(V_CHANNEL, v_pulse)
                    time.sleep_ms(20)

            except Exception as e:
                # Handle servo control errors
                force_stop_motors()
        else:
            # Target is centered, stop motors
            if motor_moving:
                force_stop_motors()
    else:
        # ====================================================================
        # NO TARGET DETECTED - STOP MOTORS
        # ====================================================================
        if motor_moving or force_stop_counter < FORCE_STOP_FRAMES:
            # Send stop commands
            set_servo_pulse(H_CHANNEL, STOP_PULSE)
            time.sleep_ms(10)
            set_servo_pulse(V_CHANNEL, STOP_PULSE)
            time.sleep_ms(10)

            force_stop_counter += 1
            if force_stop_counter >= FORCE_STOP_FRAMES:
                motor_moving = False
                last_h_pulse = STOP_PULSE
                last_v_pulse = STOP_PULSE

    # ========================================================================
    # MEMORY MANAGEMENT AND LOOP DELAY
    # ========================================================================
    # Free unused memory to prevent memory leaks
    gc.collect()
    time.sleep_ms(10)  # Small delay for system stability