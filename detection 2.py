import sensor, image, time
import pyb
import gc

# 释放内存 / Free memory
gc.collect()

# 初始化相机 / Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 获取图像尺寸 / Get image dimensions
WIDTH = sensor.width()
HEIGHT = sensor.height()
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

# PCA9685舵机控制初始化 / Initialize PCA9685 servo controller
print("初始化PCA9685舵机控制器... / Initializing PCA9685 servo controller...")
try:
    # 创建I2C对象 / Create I2C object
    i2c = pyb.I2C(2, pyb.I2C.MASTER)

    # PCA9685地址 / PCA9685 address
    PCA_ADDR = 0x40

    # 检查设备是否存在 / Check if device exists
    if PCA_ADDR not in i2c.scan():
        raise Exception("未找到PCA9685设备 / PCA9685 device not found")

    # 初始化PCA9685 / Initialize PCA9685

    # 重置 / Reset
    i2c.mem_write(0x06, PCA_ADDR, 0x00)  # MODE1寄存器软件重置
    time.sleep_ms(10)

    # 设置频率为50Hz / Set frequency to 50Hz
    i2c.mem_write(0x00, PCA_ADDR, 0x00)  # MODE1寄存器正常模式
    time.sleep_ms(10)

    # 预频率分频器 / Pre-scale value for 50Hz
    prescale = 121  # 25MHz/(4096*50Hz)-1 约等于 121
    i2c.mem_write(0x10, PCA_ADDR, 0x00)  # 进入睡眠模式
    time.sleep_ms(10)
    i2c.mem_write(prescale, PCA_ADDR, 0xFE)  # 设置预分频值
    time.sleep_ms(10)
    i2c.mem_write(0x00, PCA_ADDR, 0x00)  # 退出睡眠模式
    time.sleep_ms(10)

    print("PCA9685初始化成功 / PCA9685 initialized successfully")
except Exception as e:
    print("PCA9685初始化失败 / PCA9685 initialization failed:", e)
    print("将继续执行但舵机可能不工作 / Will continue but servos may not work")

# 舵机控制函数 / Servo control function
def set_servo_angle(channel, angle):
    """
    使用PCA9685控制舵机角度 / Control servo angle using PCA9685
    channel: 舵机通道(0-15) / Servo channel (0-15)
    angle: 角度(0-180) / Angle (0-180)
    """
    try:
        # 将角度(0-180)转换为脉冲宽度值(0-4095)
        # Convert angle (0-180) to pulse width value (0-4095)
        # 标准舵机: 0度=0.5ms, 180度=2.5ms, 中间=1.5ms
        # 20ms周期内, 0.5ms占2.5%, 2.5ms占12.5%
        # 在4096分辨率下, 2.5%=102, 12.5%=512

        # 计算脉冲宽度 / Calculate pulse width
        pulse_min = 102   # 0.5ms (0度)
        pulse_max = 512   # 2.5ms (180度)
        pulse = int(pulse_min + (pulse_max - pulse_min) * angle / 180)

        # 设置脉冲起始点(总是0) / Set pulse start point (always 0)
        i2c.mem_write(0, PCA_ADDR, 4 * channel)     # LED{n}_ON_L
        i2c.mem_write(0, PCA_ADDR, 4 * channel + 1) # LED{n}_ON_H

        # 设置脉冲结束点 / Set pulse end point
        i2c.mem_write(pulse & 0xFF, PCA_ADDR, 4 * channel + 2)       # LED{n}_OFF_L
        i2c.mem_write((pulse >> 8) & 0xFF, PCA_ADDR, 4 * channel + 3) # LED{n}_OFF_H

        return True
    except Exception as e:
        print(f"舵机控制错误 / Servo control error: {e}")
        return False

# 舵机初始位置和参数 / Initial servo position and parameters
h_angle = 90  # 水平舵机角度 / Horizontal servo angle
v_angle = 90  # 垂直舵机角度 / Vertical servo angle
h_channel = 0  # 水平舵机通道 / Horizontal servo channel (对应扩展板S0)
v_channel = 1  # 垂直舵机通道 / Vertical servo channel (对应扩展板S1)

# 设置舵机到初始位置 / Set servos to initial position
print("设置舵机到初始位置... / Setting servos to initial position...")
set_servo_angle(h_channel, h_angle)
set_servo_angle(v_channel, v_angle)
time.sleep(1)

# 舵机控制参数 / Servo control parameters
servo_step = 2       # 每次调整的角度 / Angle adjustment per step
max_angle = 180      # 最大角度 / Maximum angle
min_angle = 0        # 最小角度 / Minimum angle
target_margin = 15   # 目标区域大小 / Target area size

# 加载cascade文件 / Load cascade files
fullbody_cascade = image.HaarCascade("fullbody.cascade", stages=17)
upperbody_cascade = image.HaarCascade("haarcascade_upperbody.cascade", stages=17)

# 目标跟踪变量 / Target tracking variables
last_tracked_pos = None  # 上一帧跟踪的目标位置 / Last frame tracked position
last_tracked_type = None  # 'full' 或 'upper' / 'full' or 'upper'
track_lost_count = 0
max_lost_frames = 10
tracking_threshold = 0.4

# 计算两个矩形框的IoU / Calculate IoU between two bounding boxes
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

# 计算两个矩形框的中心点距离 / Calculate center distance between two bounding boxes
def calculate_center_distance(boxA, boxB):
    centerA_x = boxA[0] + boxA[2] // 2
    centerA_y = boxA[1] + boxA[3] // 2
    centerB_x = boxB[0] + boxB[2] // 2
    centerB_y = boxB[1] + boxB[3] // 2

    return ((centerA_x - centerB_x) ** 2 + (centerA_y - centerB_y) ** 2) ** 0.5

# 主循环 / Main loop
clock = time.clock()
while(True):
    clock.tick()

    img = sensor.snapshot()

    # 检测人体 - 先检测全身 / Detect human bodies - first try fullbody
    fullbody_objects = img.find_features(fullbody_cascade, threshold=0.75, scale_factor=1.25)

    # 检测上半身 / Detect upper body
    upperbody_objects = img.find_features(upperbody_cascade, threshold=0.75, scale_factor=1.25)

    tracked_object = None
    tracked_type = None

    # 1. 如果我们正在跟踪某个目标 / If we are currently tracking a target
    if last_tracked_pos:
        best_match = None
        best_score = 0
        best_type = None

        # 如果上一次跟踪的是全身，优先在全身检测中匹配 / If last tracked was fullbody, prioritize fullbody matching
        if last_tracked_type == 'full':
            for obj in fullbody_objects:
                iou = calculate_iou(last_tracked_pos, obj)
                distance = calculate_center_distance(last_tracked_pos, obj)

                max_distance = img.width() * 0.5
                distance_score = max(0, 1 - distance / max_distance)

                score = iou * 0.6 + distance_score * 0.4

                if score > best_score and score > tracking_threshold:
                    best_score = score
                    best_match = obj
                    best_type = 'full'

        # 如果没找到匹配的全身，或者上次跟踪的是上半身 / If no matching fullbody or last tracked was upper body
        if (not best_match and last_tracked_type == 'full') or last_tracked_type == 'upper':
            for obj in upperbody_objects:
                iou = calculate_iou(last_tracked_pos, obj)
                distance = calculate_center_distance(last_tracked_pos, obj)

                max_distance = img.width() * 0.5
                distance_score = max(0, 1 - distance / max_distance)

                score = iou * 0.6 + distance_score * 0.4

                if score > best_score and score > tracking_threshold:
                    best_score = score
                    best_match = obj
                    best_type = 'upper'

        if best_match:
            tracked_object = best_match
            tracked_type = best_type
            track_lost_count = 0
        else:
            # 没找到匹配的目标，增加丢失计数 / No matching target found, increase lost count
            track_lost_count += 1

            # 如果丢失太久，重置跟踪 / If lost for too long, reset tracking
            if track_lost_count > max_lost_frames:
                last_tracked_pos = None
                last_tracked_type = None

    # 2. 如果我们没有在跟踪目标或已丢失太久 / If we are not tracking or have lost the target
    if not last_tracked_pos or track_lost_count > max_lost_frames:
        # 优先选择全身 / Prioritize fullbody
        if fullbody_objects:
            # 选择最大的全身目标 / Select the largest fullbody target
            tracked_object = max(fullbody_objects, key=lambda r: r[2] * r[3])
            tracked_type = 'full'
            track_lost_count = 0
        # 没有全身目标，选择上半身 / No fullbody targets, select upper body
        elif upperbody_objects:
            tracked_object = max(upperbody_objects, key=lambda r: r[2] * r[3])
            tracked_type = 'upper'
            track_lost_count = 0

    # 更新跟踪状态 / Update tracking status
    if tracked_object:
        # 平滑过渡，防止抖动 / Smooth transition to prevent jitter
        if last_tracked_pos:
            alpha = 0.7  # 平滑系数 / Smoothing factor
            x = int(alpha * tracked_object[0] + (1 - alpha) * last_tracked_pos[0])
            y = int(alpha * tracked_object[1] + (1 - alpha) * last_tracked_pos[1])
            w = int(alpha * tracked_object[2] + (1 - alpha) * last_tracked_pos[2])
            h = int(alpha * tracked_object[3] + (1 - alpha) * last_tracked_pos[3])
            tracked_object = (x, y, w, h)

        # 更新上一帧目标位置和类型 / Update last frame target position and type
        last_tracked_pos = tracked_object
        last_tracked_type = tracked_type

        # 在图像上绘制当前跟踪的目标 / Draw the current tracked target on the image
        if tracked_type == 'full':
            color = (255, 0, 0)  # 全身用红色 / Red for fullbody
        else:
            color = (0, 255, 0)  # 上半身用绿色 / Green for upper body

        img.draw_rectangle(tracked_object, color=color)

        # 计算目标中心点并在图像上标记 / Calculate and mark the center point of the target
        center_x = tracked_object[0] + tracked_object[2] // 2
        center_y = tracked_object[1] + tracked_object[3] // 2
        img.draw_cross(center_x, center_y, color=color)

        # 显示跟踪类型 / Display tracking type
        img.draw_string(tracked_object[0], tracked_object[1] - 10,
                        "Full" if tracked_type == "full" else "Upper", color=color)

        # 舵机控制部分 - 控制PCA9685 / Servo control section - control PCA9685
        try:
            # 计算目标中心点与图像中心的偏差 / Calculate error between target center and image center
            x_error = CENTER_X - center_x
            y_error = CENTER_Y - center_y

            # 水平舵机控制 / Horizontal servo control
            if abs(x_error) > target_margin:
                if x_error > 0:  # 目标在左侧 / Target on left side
                    h_angle = min(max_angle, h_angle + servo_step)
                else:  # 目标在右侧 / Target on right side
                    h_angle = max(min_angle, h_angle - servo_step)
                # 设置水平舵机角度 / Set horizontal servo angle
                set_servo_angle(h_channel, h_angle)

            # 垂直舵机控制 / Vertical servo control
            if abs(y_error) > target_margin:
                if y_error > 0:  # 目标在上方 / Target on top side
                    v_angle = max(min_angle, v_angle - servo_step)
                else:  # 目标在下方 / Target on bottom side
                    v_angle = min(max_angle, v_angle + servo_step)
                # 设置垂直舵机角度 / Set vertical servo angle
                set_servo_angle(v_channel, v_angle)
        except Exception as e:
            print("舵机控制错误 / Servo control error:", e)

    # 显示当前帧率和跟踪状态 / Display fps and tracking status
    if last_tracked_pos:
        print("Tracking %s: FPS %.2f, Servo H:%d V:%d" %
             (last_tracked_type, clock.fps(), h_angle, v_angle))
    else:
        print("No target: FPS %.2f, Servo H:%d V:%d" %
             (clock.fps(), h_angle, v_angle))
