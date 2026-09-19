"""
model-310817 安装位置与两点标定测试程序。

用途：
1. 自动显示钢球检测框宽、高、等效直径和中心像素，不需要手工数像素；
2. 钢球依次静置在水管左、右两个已知位置，自动取稳定检测的中位数；
3. 输出可直接写入 calibration_config.py 的轴线与框尺寸参数。

本程序不启动RTSP、不配置Wi-Fi、不打开UART。正式比赛程序仍是 main.py。
"""

import math
import os

from maix import app, camera, display, image, nn, time


MODEL_MUD_NAME = "model_310817.mud"
BALL_CLASS_ID = 0

CAM_WIDTH = 640
CAM_HEIGHT = 360
CAM_FPS = 30
CAM_BUFFER_COUNT = 2

ROI_WIDTH = 448
ROI_HEIGHT = 252
ROI_Y = (CAM_HEIGHT - ROI_HEIGHT) // 2
SEARCH_ROIS = (
    (0, ROI_Y),
    (CAM_WIDTH - ROI_WIDTH, ROI_Y),
)

CONFIDENCE_THRESHOLD = 0.40
IOU_THRESHOLD = 0.45

# 实际有效行程：物理左端-11.5cm，物理右端+11.5cm。
POINT1_CM = -11.5
POINT2_CM = 11.5

SETUP_DELAY_MS = 3_000
STABLE_SAMPLE_COUNT = 24
STABLE_CENTER_RANGE_PX = 8.0
STABLE_BOX_RANGE_PX = 14.0
SECOND_POINT_MIN_MOVE_PX = 80.0
LIVE_LOG_INTERVAL_MS = 500


def median(values):
    """不依赖statistics模块的中位数。"""
    ordered = sorted(values)
    count = len(ordered)
    middle = count // 2
    if count % 2:
        return float(ordered[middle])
    return (float(ordered[middle - 1]) + float(ordered[middle])) * 0.5


def summarize(samples):
    """返回一组稳定观测的中心、框尺寸中位数和波动范围。"""
    return {
        "x": median([item["x"] for item in samples]),
        "y": median([item["y"] for item in samples]),
        "w": median([item["w"] for item in samples]),
        "h": median([item["h"] for item in samples]),
        "x_range": max(item["x"] for item in samples)
        - min(item["x"] for item in samples),
        "y_range": max(item["y"] for item in samples)
        - min(item["y"] for item in samples),
        "w_range": max(item["w"] for item in samples)
        - min(item["w"] for item in samples),
        "h_range": max(item["h"] for item in samples)
        - min(item["h"] for item in samples),
    }


def is_stable(samples):
    """中心与框尺寸均足够稳定才锁定标定点。"""
    if len(samples) < STABLE_SAMPLE_COUNT:
        return False
    summary = summarize(samples)
    return (
        summary["x_range"] <= STABLE_CENTER_RANGE_PX
        and summary["y_range"] <= STABLE_CENTER_RANGE_PX
        and summary["w_range"] <= STABLE_BOX_RANGE_PX
        and summary["h_range"] <= STABLE_BOX_RANGE_PX
    )


def resolve_model_path():
    app_dir = os.path.dirname(os.path.abspath(__file__))
    mud_path = os.path.join(app_dir, MODEL_MUD_NAME)
    if not os.path.isfile(mud_path):
        raise RuntimeError("Model not found: {}".format(mud_path))
    return mud_path


def load_detector():
    detector = nn.YOLOv5(
        model=resolve_model_path(),
        dual_buff=False,
    )
    if detector.labels != ["ball"]:
        raise RuntimeError(
            "Expected one ball label, got: {}".format(detector.labels)
        )
    if detector.input_width() != 448 or detector.input_height() != 448:
        raise RuntimeError(
            "Expected 448x448 model, got {}x{}".format(
                detector.input_width(),
                detector.input_height(),
            )
        )
    return detector


def best_observation(objects, roi_x, roi_y):
    """把最高置信度ball框映射到640x360全图。"""
    best = None
    for obj in objects:
        if obj.class_id != BALL_CLASS_ID:
            continue
        observation = {
            "x": roi_x + obj.x + obj.w * 0.5,
            "y": roi_y + obj.y + obj.h * 0.5,
            "bbox_x": roi_x + obj.x,
            "bbox_y": roi_y + obj.y,
            "w": float(obj.w),
            "h": float(obj.h),
            "score": float(obj.score),
        }
        if best is None or observation["score"] > best["score"]:
            best = observation
    return best


def point_distance(point_a, point_b):
    return math.sqrt(
        (point_a["x"] - point_b["x"]) ** 2
        + (point_a["y"] - point_b["y"]) ** 2
    )


def print_locked(name, point):
    diameter = (point["w"] + point["h"]) * 0.5
    print(
        (
            "{}_LOCKED center=({:.1f},{:.1f}) bbox_median={:.1f}x{:.1f}px "
            "ball_diameter~{:.1f}px center_range={:.1f}x{:.1f}px"
        ).format(
            name,
            point["x"],
            point["y"],
            point["w"],
            point["h"],
            diameter,
            point["x_range"],
            point["y_range"],
        )
    )


def print_calibration(point1, point2):
    """排序左右端点并打印正式配置建议。"""
    if point1["x"] <= point2["x"]:
        start_point, end_point = point1, point2
        start_cm, end_cm = POINT1_CM, POINT2_CM
    else:
        start_point, end_point = point2, point1
        start_cm, end_cm = POINT2_CM, POINT1_CM

    start_px = (
        int(round(start_point["x"])),
        int(round(start_point["y"])),
    )
    end_px = (
        int(round(end_point["x"])),
        int(round(end_point["y"])),
    )
    pipe_span_px = math.sqrt(
        (end_px[0] - start_px[0]) ** 2
        + (end_px[1] - start_px[1]) ** 2
    )

    endpoint_widths = [start_point["w"], end_point["w"]]
    endpoint_heights = [start_point["h"], end_point["h"]]
    smallest_side = min(endpoint_widths + endpoint_heights)
    largest_side = max(endpoint_widths + endpoint_heights)
    median_diameter = median(
        [
            (start_point["w"] + start_point["h"]) * 0.5,
            (end_point["w"] + end_point["h"]) * 0.5,
        ]
    )

    min_box = max(5, int(round(smallest_side * 0.50)))
    max_box = max(min_box + 1, int(round(largest_side * 1.80)))
    max_axis_distance = max(12, int(round(median_diameter * 0.75)))

    print("")
    print("CALIBRATION_READY")
    print(
        "MEASURED pipe_axis_span={:.1f}px ball_bbox_start={:.1f}x{:.1f}px "
        "ball_bbox_end={:.1f}x{:.1f}px ball_diameter_median~{:.1f}px".format(
            pipe_span_px,
            start_point["w"],
            start_point["h"],
            end_point["w"],
            end_point["h"],
            median_diameter,
        )
    )
    print("----- COPY_FROM_HERE -----")
    print("AXIS_START_PX = {}".format(start_px))
    print("AXIS_END_PX = {}".format(end_px))
    print("AXIS_START_CM = {:.3f}".format(start_cm))
    print("AXIS_END_CM = {:.3f}".format(end_cm))
    print("CALIBRATION_VALID = True")
    print("MIN_BALL_BOX_PX = {}".format(min_box))
    print("MAX_BALL_BOX_PX = {}".format(max_box))
    print("MAX_BALL_ASPECT_RATIO = 1.80")
    print("MAX_AXIS_DISTANCE_PX = {:.1f}".format(max_axis_distance))
    print("AXIS_RATIO_MARGIN = 0.08")
    print("----- COPY_TO_HERE -----")
    print(
        "[IMPORTANT] Send the whole block and MEASURED line to Codex; "
        "do not estimate pixels manually."
    )


def draw_grid(frame):
    """50像素网格只用于理解位置，不要求人工计数。"""
    grid_color = image.Color.from_rgb(70, 70, 70)
    for x in range(0, CAM_WIDTH, 50):
        frame.draw_line(
            x,
            0,
            x,
            CAM_HEIGHT - 1,
            color=grid_color,
            thickness=1,
        )
    for y in range(0, CAM_HEIGHT, 50):
        frame.draw_line(
            0,
            y,
            CAM_WIDTH - 1,
            y,
            color=grid_color,
            thickness=1,
        )


def draw_overlay(frame, roi_x, observation, phase, sample_count, point1):
    draw_grid(frame)
    frame.draw_rect(
        roi_x,
        ROI_Y,
        ROI_WIDTH,
        ROI_HEIGHT,
        color=image.COLOR_BLUE,
        thickness=2,
    )
    frame.draw_line(
        CAM_WIDTH // 2,
        0,
        CAM_WIDTH // 2,
        CAM_HEIGHT - 1,
        color=image.COLOR_YELLOW,
        thickness=1,
    )

    if point1 is not None:
        frame.draw_cross(
            int(round(point1["x"])),
            int(round(point1["y"])),
            image.COLOR_YELLOW,
            12,
            2,
        )

    if observation is not None:
        frame.draw_rect(
            int(observation["bbox_x"]),
            int(observation["bbox_y"]),
            int(observation["w"]),
            int(observation["h"]),
            color=image.COLOR_RED,
            thickness=2,
        )
        frame.draw_cross(
            int(round(observation["x"])),
            int(round(observation["y"])),
            image.COLOR_GREEN,
            10,
            2,
        )
        text = "x={:.1f} y={:.1f} box={:.0f}x{:.0f}px".format(
            observation["x"],
            observation["y"],
            observation["w"],
            observation["h"],
        )
    else:
        text = "BALL NOT FOUND"

    frame.draw_rect(
        0,
        0,
        CAM_WIDTH,
        46,
        image.Color.from_rgb(0, 0, 0),
        -1,
    )
    frame.draw_string(
        6,
        4,
        "{} samples={}/{}".format(
            phase,
            sample_count,
            STABLE_SAMPLE_COUNT,
        ),
        color=image.COLOR_WHITE,
        scale=1.0,
        thickness=1,
    )
    frame.draw_string(
        6,
        24,
        text,
        color=image.COLOR_WHITE,
        scale=1.0,
        thickness=1,
    )


def main():
    print("Program: model_310817_calibration_p17_v1")
    print(
        "Before start: mount camera firmly and put ball at the LEFT known endpoint."
    )
    print(
        "Step 1: keep point 1 still until POINT1_LOCKED; "
        "Step 2: move to RIGHT endpoint until CALIBRATION_READY."
    )
    print(
        "The program prints center and bbox pixels automatically; no manual counting."
    )

    detector = load_detector()
    cam = camera.Camera(
        CAM_WIDTH,
        CAM_HEIGHT,
        image.Format.FMT_RGB888,
        fps=CAM_FPS,
        buff_num=CAM_BUFFER_COUNT,
    )
    disp = display.Display()

    phase = "SET_POINT1"
    samples = []
    point1 = None
    point2 = None
    roi_index = 0
    start_ms = time.ticks_ms()
    last_log_ms = start_ms

    try:
        while not app.need_exit():
            frame = cam.read()
            roi_x, roi_y = SEARCH_ROIS[roi_index]
            roi_index = (roi_index + 1) % len(SEARCH_ROIS)
            roi_frame = frame.crop(
                roi_x,
                roi_y,
                ROI_WIDTH,
                ROI_HEIGHT,
            )
            objects = detector.detect(
                roi_frame,
                conf_th=CONFIDENCE_THRESHOLD,
                iou_th=IOU_THRESHOLD,
            )
            observation = best_observation(objects, roi_x, roi_y)
            now_ms = time.ticks_ms()

            if (
                observation is not None
                and now_ms - start_ms >= SETUP_DELAY_MS
                and point2 is None
            ):
                if point1 is None:
                    samples.append(observation)
                    samples = samples[-STABLE_SAMPLE_COUNT:]
                    if is_stable(samples):
                        point1 = summarize(samples)
                        print_locked("POINT1", point1)
                        print(
                            "Move the ball to the other known endpoint "
                            "(at least {}px away) and hold still.".format(
                                int(SECOND_POINT_MIN_MOVE_PX)
                            )
                        )
                        samples = []
                        phase = "MOVE_TO_POINT2"
                elif point_distance(observation, point1) >= SECOND_POINT_MIN_MOVE_PX:
                    phase = "SET_POINT2"
                    samples.append(observation)
                    samples = samples[-STABLE_SAMPLE_COUNT:]
                    if is_stable(samples):
                        point2 = summarize(samples)
                        print_locked("POINT2", point2)
                        print_calibration(point1, point2)
                        phase = "DONE"

            if now_ms - last_log_ms >= LIVE_LOG_INTERVAL_MS:
                if observation is None:
                    print("LIVE ball_not_found phase={}".format(phase))
                else:
                    print(
                        (
                            "LIVE center=({:.1f},{:.1f}) bbox={:.0f}x{:.0f}px "
                            "diameter~{:.1f}px score={:.3f} phase={}"
                        ).format(
                            observation["x"],
                            observation["y"],
                            observation["w"],
                            observation["h"],
                            (observation["w"] + observation["h"]) * 0.5,
                            observation["score"],
                            phase,
                        )
                    )
                last_log_ms = now_ms

            draw_overlay(
                frame,
                roi_x,
                observation,
                phase,
                len(samples),
                point1,
            )
            disp.show(frame)
    finally:
        cam.close()


if __name__ == "__main__":
    main()
