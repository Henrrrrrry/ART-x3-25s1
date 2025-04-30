import sensor, image, time
import gc

# 释放内存
gc.collect()

# 初始化相机
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 加载cascade文件
fullbody_cascade = image.HaarCascade("fullbody.cascade",stages=17)
upperbody_cascade = image.HaarCascade("haarcascade_upperbody.cascade",stages=17)

# 目标跟踪变量
last_tracked_pos = None  # 上一帧跟踪的目标位置 (x, y, w, h)
last_tracked_type = None  # 'full' 或 'upper'
track_lost_count = 0
max_lost_frames = 10
tracking_threshold = 0.4

# 计算两个矩形框的IoU
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

# 计算两个矩形框的中心点距离
def calculate_center_distance(boxA, boxB):
    centerA_x = boxA[0] + boxA[2] // 2
    centerA_y = boxA[1] + boxA[3] // 2
    centerB_x = boxB[0] + boxB[2] // 2
    centerB_y = boxB[1] + boxB[3] // 2

    return ((centerA_x - centerB_x) ** 2 + (centerA_y - centerB_y) ** 2) ** 0.5

# 主循环
clock = time.clock()
while(True):
    clock.tick()

    img = sensor.snapshot()

    # 检测人体 - 先检测全身
    fullbody_objects = img.find_features(fullbody_cascade, threshold=0.75, scale_factor=1.25)

    # 检测上半身
    upperbody_objects = img.find_features(upperbody_cascade, threshold=0.75, scale_factor=1.25)

    tracked_object = None
    tracked_type = None

    # 1. 如果我们正在跟踪某个目标
    if last_tracked_pos:
        best_match = None
        best_score = 0
        best_type = None

        # 如果上一次跟踪的是全身，优先在全身检测中匹配
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

        # 如果没找到匹配的全身，或者上次跟踪的是上半身
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
            # 没找到匹配的目标，增加丢失计数
            track_lost_count += 1

            # 如果丢失太久，重置跟踪
            if track_lost_count > max_lost_frames:
                last_tracked_pos = None
                last_tracked_type = None

    # 2. 如果我们没有在跟踪目标或已丢失太久
    if not last_tracked_pos or track_lost_count > max_lost_frames:
        # 优先选择全身
        if fullbody_objects:
            # 选择最大的全身目标
            tracked_object = max(fullbody_objects, key=lambda r: r[2] * r[3])
            tracked_type = 'full'
            track_lost_count = 0
        # 没有全身目标，选择上半身
        elif upperbody_objects:
            tracked_object = max(upperbody_objects, key=lambda r: r[2] * r[3])
            tracked_type = 'upper'
            track_lost_count = 0

    # 更新跟踪状态
    if tracked_object:
        # 平滑过渡，防止抖动
        if last_tracked_pos:
            alpha = 0.7  # 平滑系数
            x = int(alpha * tracked_object[0] + (1 - alpha) * last_tracked_pos[0])
            y = int(alpha * tracked_object[1] + (1 - alpha) * last_tracked_pos[1])
            w = int(alpha * tracked_object[2] + (1 - alpha) * last_tracked_pos[2])
            h = int(alpha * tracked_object[3] + (1 - alpha) * last_tracked_pos[3])
            tracked_object = (x, y, w, h)

        # 更新上一帧目标位置和类型
        last_tracked_pos = tracked_object
        last_tracked_type = tracked_type

        # 在图像上绘制当前跟踪的目标
        if tracked_type == 'full':
            color = (255, 0, 0)  # 全身用红色
        else:
            color = (0, 255, 0)  # 上半身用绿色

        img.draw_rectangle(tracked_object, color=color)

        # 计算目标中心点并在图像上标记
        center_x = tracked_object[0] + tracked_object[2] // 2
        center_y = tracked_object[1] + tracked_object[3] // 2
        img.draw_cross(center_x, center_y, color=color)

        # 显示跟踪类型
        img.draw_string(tracked_object[0], tracked_object[1] - 10,
                        "Full" if tracked_type == "full" else "Upper", color=color)

    # 显示当前帧率和跟踪状态
    if last_tracked_pos:
        print("Tracking %s: FPS %.2f" % (last_tracked_type, clock.fps()))
    else:
        print("No target: FPS %.2f" % clock.fps())
