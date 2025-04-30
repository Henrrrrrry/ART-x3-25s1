import sensor, image, time
import gc

# Release memory
gc.collect()

# Initialize the camera
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# Load the cascade file
fullbody_cascade = image.HaarCascade("fullbody.cascade",stages=17)
upperbody_cascade = image.HaarCascade("haarcascade_upperbody.cascade",stages=17)

# Target tracking variable
last_tracked_pos = None  # 上一帧跟踪的目标位置 (x, y, w, h)
last_tracked_type = None  # 'full' 或 'upper'
track_lost_count = 0
max_lost_frames = 10
tracking_threshold = 0.4

# Calculate the IoU of the two rectangular boxes
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

# Calculate the distance between the center points of the two rectangular boxes
def calculate_center_distance(boxA, boxB):
    centerA_x = boxA[0] + boxA[2] // 2
    centerA_y = boxA[1] + boxA[3] // 2
    centerB_x = boxB[0] + boxB[2] // 2
    centerB_y = boxB[1] + boxB[3] // 2

    return ((centerA_x - centerB_x) ** 2 + (centerA_y - centerB_y) ** 2) ** 0.5

# Main cycle
clock = time.clock()
while(True):
    clock.tick()

    img = sensor.snapshot()

    # Examine the human body - Examine the entire body first
    fullbody_objects = img.find_features(fullbody_cascade, threshold=0.75, scale_factor=1.25)

    # Test the upper body
    upperbody_objects = img.find_features(upperbody_cascade, threshold=0.75, scale_factor=1.25)

    tracked_object = None
    tracked_type = None

    # 1. If we are tracking a certain target
    if last_tracked_pos:
        best_match = None
        best_score = 0
        best_type = None

        # If the last tracking was full-body, it should be matched in the full-body test first
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

        # If no matching full body is found, or the upper body was tracked last time
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

            # If lost for too long, reset the tracking
            if track_lost_count > max_lost_frames:
                last_tracked_pos = None
                last_tracked_type = None

    # 2. If we are not tracking the target or it has been lost for too long
    if not last_tracked_pos or track_lost_count > max_lost_frames:
        # Give priority to the whole body.
        if fullbody_objects:
            # Choose the largest full-body target
            tracked_object = max(fullbody_objects, key=lambda r: r[2] * r[3])
            tracked_type = 'full'
            track_lost_count = 0
        # If there is no full-body target, choose the upper body
        elif upperbody_objects:
            tracked_object = max(upperbody_objects, key=lambda r: r[2] * r[3])
            tracked_type = 'upper'
            track_lost_count = 0

    # Update the tracking status
    if tracked_object:
        # Smooth transition to prevent jitter
        if last_tracked_pos:
            alpha = 0.7  # Smoothness coefficient
            x = int(alpha * tracked_object[0] + (1 - alpha) * last_tracked_pos[0])
            y = int(alpha * tracked_object[1] + (1 - alpha) * last_tracked_pos[1])
            w = int(alpha * tracked_object[2] + (1 - alpha) * last_tracked_pos[2])
            h = int(alpha * tracked_object[3] + (1 - alpha) * last_tracked_pos[3])
            tracked_object = (x, y, w, h)

        # Update the target position and type of the previous frame
        last_tracked_pos = tracked_object
        last_tracked_type = tracked_type

        # Draw the currently tracked target on the image
        if tracked_type == 'full':
            color = (255, 0, 0)  # red all over the body.
        else:
            color = (0, 255, 0)  # green for the upper body

        img.draw_rectangle(tracked_object, color=color)

        # Calculate the center point of the target and mark it on the image
        center_x = tracked_object[0] + tracked_object[2] // 2
        center_y = tracked_object[1] + tracked_object[3] // 2
        img.draw_cross(center_x, center_y, color=color)

        # Display tracking type
        img.draw_string(tracked_object[0], tracked_object[1] - 10,
                        "Full" if tracked_type == "full" else "Upper", color=color)

    # Display the current frame rate and tracking status
    if last_tracked_pos:
        print("Tracking %s: FPS %.2f" % (last_tracked_type, clock.fps()))
    else:
        print("No target: FPS %.2f" % clock.fps())
