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
sensor.set_framerate(20)

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
i2c = I2C(2, freq=100000)

# PCA9685 initialization function
def reset_pca9685():
    try:
        mode1 = i2c.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 | 0x10]))
        time.sleep(0.005)

        prescale = 25000000 // (4096 * 50) - 1
        i2c.writeto_mem(PCA9685_ADDR, PRESCALE, bytes([prescale]))
        time.sleep(0.005)

        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10]))
        time.sleep(0.005)

        i2c.writeto_mem(PCA9685_ADDR, MODE1, bytes([mode1 & ~0x10 | 0xa0]))
        time.sleep(0.005)

        return True
    except Exception as e:
        return False

# Servo control function
def set_servo_pulse(channel, pulse_us):
    try:
        pulse_count = pulse_us * 4096 // 20000
        channel_base = LED0_ON_L + (channel * 4)

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
STOP_PULSE = 1520
FORWARD_PULSE = 1530
REVERSE_PULSE = 1455
SLOW_FORWARD = 1525
SLOW_REVERSE = 1475

# Servo channels
H_CHANNEL = 0
V_CHANNEL = 1

# Load cascade file
upperbody_cascade = image.HaarCascade("haarcascade_upperbody.cascade", stages=17)

# Tracking parameters
SMALL_ERROR = 15
LARGE_ERROR = 40

#
MAX_LOST_FRAMES = 3


FORCE_STOP_FRAMES = 5
force_stop_counter = 0

# Set servos to initial position
set_servo_pulse(H_CHANNEL, STOP_PULSE)
time.sleep_ms(100)
set_servo_pulse(V_CHANNEL, STOP_PULSE)
time.sleep(1)

# Target tracking variables
last_tracked_pos = None
track_lost_count = 0

# Motor status variables
last_h_pulse = STOP_PULSE
last_v_pulse = STOP_PULSE

# motor tracking
motor_moving = False
last_detection_time = time.ticks_ms()

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

# force stop
def force_stop_motors():

    global force_stop_counter, motor_moving, last_h_pulse, last_v_pulse


    for i in range(3):
        set_servo_pulse(H_CHANNEL, STOP_PULSE)
        time.sleep_ms(10)
        set_servo_pulse(V_CHANNEL, STOP_PULSE)
        time.sleep_ms(10)

    last_h_pulse = STOP_PULSE
    last_v_pulse = STOP_PULSE
    motor_moving = False
    force_stop_counter = 0

# Main loop
clock = time.clock()

while(True):
    clock.tick()

    # Capture image
    img = sensor.snapshot()

    # Detect upper body
    upperbody_objects = img.find_features(upperbody_cascade, threshold=0.70, scale_factor=1.2)

    tracked_object = None
    current_time = time.ticks_ms()

    # 1. If we are tracking a target
    if last_tracked_pos:
        best_match = None
        best_score = 0

        for obj in upperbody_objects:
            iou = calculate_iou(last_tracked_pos, obj)
            if iou > best_score and iou > 0.3:
                best_score = iou
                best_match = obj

        if best_match:
            tracked_object = best_match
            track_lost_count = 0
            last_detection_time = current_time
        else:
            track_lost_count += 1

            # faster force stop
            if track_lost_count > MAX_LOST_FRAMES:
                last_tracked_pos = None
                force_stop_motors()  #froce stop

    # 2. If we are not tracking a target
    if not last_tracked_pos:
        if upperbody_objects:
            tracked_object = max(upperbody_objects, key=lambda r: r[2] * r[3])
            track_lost_count = 0
            last_detection_time = current_time
        else:
            # check if there's need to stop
            time_since_detection = time.ticks_diff(current_time, last_detection_time)
            if time_since_detection > 500:
                if motor_moving:
                    force_stop_motors()

    # 3. Control servos
    if tracked_object:
        # Smooth transition
        if last_tracked_pos:
            alpha = 0.7
            x = int(alpha * tracked_object[0] + (1 - alpha) * last_tracked_pos[0])
            y = int(alpha * tracked_object[1] + (1 - alpha) * last_tracked_pos[1])
            w = int(alpha * tracked_object[2] + (1 - alpha) * last_tracked_pos[2])
            h = int(alpha * tracked_object[3] + (1 - alpha) * last_tracked_pos[3])
            tracked_object = (x, y, w, h)

        last_tracked_pos = tracked_object

        # Draw tracking info
        img.draw_rectangle(tracked_object, color=(255, 0, 0))
        center_x = tracked_object[0] + tracked_object[2] // 2
        center_y = tracked_object[1] + tracked_object[3] // 2
        img.draw_cross(center_x, center_y, color=(255, 0, 0))

        # Calculate errors
        x_error = CENTER_X - center_x
        y_error = CENTER_Y - center_y

        # rotate when error is big
        should_move = abs(x_error) > SMALL_ERROR or abs(y_error) > SMALL_ERROR

        if should_move:
            motor_moving = True
            force_stop_counter = 0

            try:
                # Horizontal control
                h_pulse = STOP_PULSE
                if abs(x_error) > SMALL_ERROR:
                    if x_error > 0:
                        h_pulse = FORWARD_PULSE if abs(x_error) > LARGE_ERROR else SLOW_FORWARD
                    else:
                        h_pulse = REVERSE_PULSE if abs(x_error) > LARGE_ERROR else SLOW_REVERSE

                if h_pulse != last_h_pulse:
                    last_h_pulse = h_pulse
                    set_servo_pulse(H_CHANNEL, h_pulse)
                    time.sleep_ms(20)

                # Vertical control
                v_pulse = STOP_PULSE
                if abs(y_error) > SMALL_ERROR:
                    if y_error > 0:
                        v_pulse = REVERSE_PULSE if abs(y_error) > LARGE_ERROR else SLOW_REVERSE
                    else:
                        v_pulse = FORWARD_PULSE if abs(y_error) > LARGE_ERROR else SLOW_FORWARD

                if v_pulse != last_v_pulse:
                    last_v_pulse = v_pulse
                    set_servo_pulse(V_CHANNEL, v_pulse)
                    time.sleep_ms(20)

            except Exception as e:
                force_stop_motors()
        else:
            # stop in center
            if motor_moving:
                force_stop_motors()
    else:
        # if no target
        if motor_moving or force_stop_counter < FORCE_STOP_FRAMES:
            # stop
            set_servo_pulse(H_CHANNEL, STOP_PULSE)
            time.sleep_ms(10)
            set_servo_pulse(V_CHANNEL, STOP_PULSE)
            time.sleep_ms(10)

            force_stop_counter += 1
            if force_stop_counter >= FORCE_STOP_FRAMES:
                motor_moving = False
                last_h_pulse = STOP_PULSE
                last_v_pulse = STOP_PULSE

    # Free memory
    gc.collect()
    time.sleep_ms(10)
