import sensor, image, time
import gc
from machine import I2C

# Enable memory management
gc.enable()
gc.collect()

# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_framerate(20)  # Set appropriate frame rate

# Get image dimensions
WIDTH = sensor.width()
HEIGHT = sensor.height()
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

# PCA9685 register addresses
PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# Create I2C object
i2c = I2C(2, freq=100000)  # Set lower frequency for better stability

# PCA9685 initialization function
def reset_pca9685():
    try:
        # Read current MODE1 register value
        mode1 = i2c.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]

        # Set sleep mode
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 | 0x10]))
        time.sleep(0.005)

        # Set PWM frequency (50Hz)
        prescale = 25000000 // (4096 * 50) - 1
        i2c.writeto_mem(PCA9685_ADDR, PRESCALE, bytes([prescale]))
        time.sleep(0.005)

        # Restore normal mode
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10]))
        time.sleep(0.005)  # Wait for oscillator to start

        # Enable auto-increment
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10 | 0xa0]))
        time.sleep(0.005)

        return True
    except Exception as e:
        return False

# Servo control function - write separately for better stability
def set_servo_pulse(channel, pulse_us):
    try:
        # Convert pulse width (microseconds) to 12-bit count value
        pulse_count = pulse_us * 4096 // 20000

        # Calculate register values
        channel_base = LED0_ON_L + (channel * 4)

        # Write byte by byte for better stability
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

# Initialize PCA9685
reset_status = reset_pca9685()

# Servo parameters
STOP_PULSE = 1530    # Stop pulse width
FORWARD_PULSE = 1540 # Forward rotation pulse width
REVERSE_PULSE = 1460 # Reverse rotation pulse width

# Speed control parameters
SLOW_FORWARD = 1535  # Slow forward
SLOW_REVERSE = 1490  # Slow reverse

# Servo channels
H_CHANNEL = 0  # Horizontal servo channel (corresponds to extension board S0)
V_CHANNEL = 1  # Vertical servo channel (corresponds to extension board S1)

# Load cascade file
upperbody_cascade = image.HaarCascade("haarcascade_upperbody.cascade", stages=17)

# Tracking parameters
SMALL_ERROR = 15   # Small error threshold
LARGE_ERROR = 40   # Large error threshold

# Set servos to initial position
set_servo_pulse(H_CHANNEL, STOP_PULSE)
time.sleep_ms(100)
set_servo_pulse(V_CHANNEL, STOP_PULSE)
time.sleep(1)

# Target tracking variables
last_tracked_pos = None  # Last frame's target position
track_lost_count = 0
max_lost_frames = 10

# Motor status variables
last_h_pulse = STOP_PULSE
last_v_pulse = STOP_PULSE

# Calculate IoU between two bounding boxes
def calculate_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
    yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = boxA[2] * boxA[3]
    boxBArea = boxB[2] * boxB[3]

    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

# Main loop
clock = time.clock()

while(True):
    clock.tick()

    # Capture image
    img = sensor.snapshot()

    # Detect upper body
    upperbody_objects = img.find_features(upperbody_cascade, threshold=0.75, scale_factor=1.25)

    tracked_object = None

    # 1. If we are tracking a target
    if last_tracked_pos:
        best_match = None
        best_score = 0

        # Find matching target in upper body detection results
        for obj in upperbody_objects:
            iou = calculate_iou(last_tracked_pos, obj)
            if iou > best_score and iou > 0.3:
                best_score = iou
                best_match = obj

        if best_match:
            tracked_object = best_match
            track_lost_count = 0
        else:
            # No matching target found, increment lost count
            track_lost_count += 1

            # Reset tracking if lost for too long
            if track_lost_count > max_lost_frames:
                last_tracked_pos = None

    # 2. If we are not tracking a target or have lost it for too long
    if not last_tracked_pos or track_lost_count > max_lost_frames:
        # Select an upper body target
        if upperbody_objects:
            # Select the largest target
            tracked_object = max(upperbody_objects, key=lambda r: r[2] * r[3])
            track_lost_count = 0

    # 3. Update tracking status and control servos
    if tracked_object:
        # Smooth transition to prevent jitter
        if last_tracked_pos:
            alpha = 0.7  # Smoothing coefficient
            x = int(alpha * tracked_object[0] + (1 - alpha) * last_tracked_pos[0])
            y = int(alpha * tracked_object[1] + (1 - alpha) * last_tracked_pos[1])
            w = int(alpha * tracked_object[2] + (1 - alpha) * last_tracked_pos[2])
            h = int(alpha * tracked_object[3] + (1 - alpha) * last_tracked_pos[3])
            tracked_object = (x, y, w, h)

        # Update last frame's target position
        last_tracked_pos = tracked_object

        # Draw current tracked target on image
        img.draw_rectangle(tracked_object, color=(255, 0, 0))

        # Calculate target center point and mark on image
        center_x = tracked_object[0] + tracked_object[2] // 2
        center_y = tracked_object[1] + tracked_object[3] // 2
        img.draw_cross(center_x, center_y, color=(255, 0, 0))

        # Calculate deviation between target center and image center
        x_error = CENTER_X - center_x
        y_error = CENTER_Y - center_y

        try:
            # Horizontal servo control (S0)
            h_pulse = STOP_PULSE  # Default stop

            if abs(x_error) > SMALL_ERROR:
                if x_error > 0:  # Target on the left
                    if abs(x_error) > LARGE_ERROR:
                        h_pulse = FORWARD_PULSE  # Fast left movement
                    else:
                        h_pulse = SLOW_FORWARD   # Slow left movement
                else:  # Target on the right
                    if abs(x_error) > LARGE_ERROR:
                        h_pulse = REVERSE_PULSE  # Fast right movement
                    else:
                        h_pulse = SLOW_REVERSE   # Slow right movement

            # Update pulse value if changed
            if h_pulse != last_h_pulse:
                last_h_pulse = h_pulse

            # Set horizontal servo
            set_servo_pulse(H_CHANNEL, h_pulse)

            # Important delay to ensure command completion
            time.sleep_ms(20)

            # Vertical servo control (S1)
            v_pulse = STOP_PULSE  # Default stop

            if abs(y_error) > SMALL_ERROR:
                if y_error > 0:  # Target above
                    if abs(y_error) > LARGE_ERROR:
                        v_pulse = REVERSE_PULSE  # Fast upward movement
                    else:
                        v_pulse = SLOW_REVERSE   # Slow upward movement
                else:  # Target below
                    if abs(y_error) > LARGE_ERROR:
                        v_pulse = FORWARD_PULSE  # Fast downward movement
                    else:
                        v_pulse = SLOW_FORWARD   # Slow downward movement

            # Update pulse value if changed
            if v_pulse != last_v_pulse:
                last_v_pulse = v_pulse

            # Set vertical servo
            set_servo_pulse(V_CHANNEL, v_pulse)

            # Important delay to ensure command completion
            time.sleep_ms(20)

        except Exception as e:
            pass
    else:
        # Stop servos immediately when no target is detected
        # Always set to STOP_PULSE regardless of previous state
        set_servo_pulse(H_CHANNEL, 1530)
        time.sleep_ms(20)
        set_servo_pulse(V_CHANNEL, 1530)
        last_h_pulse = STOP_PULSE
        last_v_pulse = STOP_PULSE

    # Free memory
    gc.collect()

    # Short delay to reduce system load
    time.sleep_ms(10)
