"""
MaixCAM Pro model-310817 1280x720 WebRTC H.264 + ROI识球 + UART1。

主通道：1280x720 NV21，绑定WebRTC硬件H.264并输出完整原始画面。
副通道：640x360 RGB888，每帧裁剪一个448x252的16:9窗口供AI。

YOLOv5开启dual_buff提高吞吐量。detect()返回上一轮输入的结果，因此程序
保存上一轮ROI元数据后再做全图坐标回映射，防止结果映射到错误窗口。

串口帧：0x7B + tracking + int16_be(dx_uart) + 0x7D。
程序创建或复用固定MaixCAM热点，平板无需手机热点或外部路由器。
"""

import os

from maix import (
    app,
    camera,
    display,
    err,
    image,
    network,
    nn,
    pinmap,
    sys,
    time,
    uart,
    webrtc,
)

from ball_position import (
    AdaptiveAlphaBetaFilter,
    SelectedZeroManager,
    axis_point,
    position_from_pixel,
    validate_calibration,
)
from calibration_config import (
    AXIS_END_CM,
    AXIS_END_PX,
    AXIS_RATIO_MARGIN,
    AXIS_START_CM,
    AXIS_START_PX,
    CALIBRATION_VALID,
    MAX_AXIS_DISTANCE_PX,
    MAX_BALL_ASPECT_RATIO,
    MAX_BALL_BOX_PX,
    MIN_BALL_BOX_PX,
    SELECTED_ZERO_CM_DEFAULT,
    UART_UNITS_PER_CM,
    ZERO_MAX_POSITION_SPAN_CM,
    ZERO_STABLE_SAMPLE_COUNT,
)
from webrtc_record_server import (
    RECORD_HTTP_PORT,
    claim_zero_request,
    complete_zero_request,
    start_record_server,
    stop_record_server,
    update_zero_runtime_status,
)


PROGRAM_VERSION = "webrtc_ball_hdemo_uart_310817_roi_selected_zero_v1"

MODEL_MUD_NAME = "model_310817.mud"
MODEL_FILE_NAME = "model_310817.cvimodel"
BALL_CLASS_ID = 0
MODEL_WIDTH = 448
MODEL_HEIGHT = 448

AI_WIDTH = 640
AI_HEIGHT = 360
AI_FPS = 30
AI_BUFFER_COUNT = 2

WEBRTC_WIDTH = 1280
WEBRTC_HEIGHT = 720
WEBRTC_FPS = 30
WEBRTC_BUFFER_COUNT = 3
WEBRTC_PORT = 8000
WEBRTC_SIGNALING_PORT = 8001
WEBRTC_BITRATE = 2_500_000
WEBRTC_GOP = 30

# MaixCAM Pro固定2.4GHz热点；已是正确AP时直接复用，不重启无线接口。
WIFI_AP_SSID = "MaixBall-AP"
try:
    from local_config import WIFI_AP_PASSWORD
except ImportError:
    WIFI_AP_PASSWORD = None
WIFI_AP_MODE = "g"
WIFI_AP_CHANNEL = 6
WIFI_AP_IP = "192.168.66.1"
WIFI_AP_NETMASK = "255.255.255.0"
WIFI_AP_HIDDEN = False
WIFI_AP_START_TIMEOUT_MS = 3_000

# 训练图片以16:9画面等比缩放到448x448。运行时也使用16:9 ROI，
# 保持训练和推理的画面比例一致。左右窗口重叠256像素并覆盖完整640宽。
ROI_WIDTH = 448
ROI_HEIGHT = 252
ROI_Y = (AI_HEIGHT - ROI_HEIGHT) // 2
SEARCH_ROIS = (
    (0, ROI_Y),
    (AI_WIDTH - ROI_WIDTH, ROI_Y),
)

TRACK_MISS_LIMIT = 3
TRACK_MAX_JUMP_PX = 160
RESULT_MAX_AGE_MS = 250

CONFIDENCE_THRESHOLD = 0.40
IOU_THRESHOLD = 0.45

# dual_buff=True会提高吞吐量，但detect()结果对应上一次调用的图像。
ENABLE_DUAL_BUFFER = True
ENABLE_DISPLAY = False
VERBOSE_RESULT_LOG = False

UART_DEVICE = "/dev/ttyS1"
UART_TX_PIN = "A19"
UART_RX_PIN = "A18"
UART_BAUDRATE = 115200
UART_FRAME_HEADER = 0x7B
UART_FRAME_TAIL = 0x7D

# 厘米误差映射到协议安全范围，保证两个数据字节不出现帧定界符。
UART_ERROR_MIN = -112
UART_ERROR_MAX = 111

READ_RETRY_DELAY_MS = 10
STATS_INTERVAL_MS = 1_000


def print_runtime_info():
    """打印设备、模型、WebRTC、AI、ROI、标定和串口信息。"""
    print("Program:", PROGRAM_VERSION)
    print("Device:", sys.device_name())
    print("OS:", sys.os_version())
    print("MaixPy:", sys.maixpy_version())
    print("IP addresses:", sys.ip_address())
    print(
        "WebRTC: {}x{} NV21 H264 @ {} fps, bitrate={}, gop={}".format(
            WEBRTC_WIDTH,
            WEBRTC_HEIGHT,
            WEBRTC_FPS,
            WEBRTC_BITRATE,
            WEBRTC_GOP,
        )
    )
    print(
        "AI channel: {}x{} RGB888 @ {} fps".format(
            AI_WIDTH,
            AI_HEIGHT,
            AI_FPS,
        )
    )
    print(
        "AI ROI: {}x{}, search_rois={}, dual_buff={}".format(
            ROI_WIDTH,
            ROI_HEIGHT,
            SEARCH_ROIS,
            ENABLE_DUAL_BUFFER,
        )
    )
    print(
        "Calibration(valid={}): {}={}cm, {}={}cm".format(
            CALIBRATION_VALID,
            AXIS_START_PX,
            AXIS_START_CM,
            AXIS_END_PX,
            AXIS_END_CM,
        )
    )
    print(
        (
            "Detection filters: box={}..{}px aspect<={:.2f}; "
            "axis_distance<={:.1f}px ratio_margin={:.2f}"
        ).format(
            MIN_BALL_BOX_PX,
            MAX_BALL_BOX_PX,
            MAX_BALL_ASPECT_RATIO,
            MAX_AXIS_DISTANCE_PX,
            AXIS_RATIO_MARGIN,
        )
    )
    if CALIBRATION_VALID:
        print("[OK] Axis-distance filter enabled")
    else:
        print(
            "[IMPORTANT] Calibration not measured: axis-distance filter bypassed; "
            "run calibrate.py"
        )
    print(
        "UART: {} {} 8N1 TX={} RX={}".format(
            UART_DEVICE,
            UART_BAUDRATE,
            UART_TX_PIN,
            UART_RX_PIN,
        )
    )
    print("UART packet: 7B tracking dx_hi dx_lo 7D")
    print(
        "UART error: (position_cm-selected_zero_cm)*{:.1f}, "
        "clamp={}..{}, default_zero={:.2f}cm".format(
            UART_UNITS_PER_CM,
            UART_ERROR_MIN,
            UART_ERROR_MAX,
            SELECTED_ZERO_CM_DEFAULT,
        )
    )
    print(
        "Training report: input=448x448, best_epoch=200, "
        "best_val_acc=1.0, displayed_validation_images=3"
    )


def create_wifi_ap():
    if not WIFI_AP_PASSWORD or len(WIFI_AP_PASSWORD) < 8:
        raise RuntimeError(
            "Copy local_config.py.example to local_config.py and set a WiFi "
            "password of at least 8 characters"
        )

    """复用配置正确的AP；只在必要时断开STA并创建固定热点。"""
    wifi_dev = network.wifi.Wifi()

    if wifi_dev.is_ap_mode():
        current_ip = wifi_dev.get_ip()
        if current_ip == WIFI_AP_IP:
            print(
                "WiFi AP reused: ssid={}, ip={}, no restart".format(
                    WIFI_AP_SSID,
                    current_ip,
                )
            )
            print("IP addresses in AP mode:", sys.ip_address())
            return wifi_dev

        print(
            "[WARN] Existing AP IP {} differs from {}, restarting AP".format(
                current_ip or "<empty>",
                WIFI_AP_IP,
            )
        )
        err.check_raise(
            wifi_dev.stop_ap(),
            "Failed to stop existing WiFi AP",
        )

    if wifi_dev.is_connected():
        err.check_raise(
            wifi_dev.disconnect(),
            "Failed to disconnect WiFi STA",
        )

    err.check_raise(
        wifi_dev.start_ap(
            WIFI_AP_SSID,
            WIFI_AP_PASSWORD,
            mode=WIFI_AP_MODE,
            channel=WIFI_AP_CHANNEL,
            ip=WIFI_AP_IP,
            netmask=WIFI_AP_NETMASK,
            hidden=WIFI_AP_HIDDEN,
        ),
        "Failed to start WiFi AP",
    )

    wait_start_ms = time.ticks_ms()
    while not wifi_dev.is_ap_mode():
        if time.ticks_ms() - wait_start_ms >= WIFI_AP_START_TIMEOUT_MS:
            raise RuntimeError("WiFi AP did not become ready before timeout")
        time.sleep_ms(100)

    print(
        "WiFi AP: ssid={}, channel={}, ip={}".format(
            WIFI_AP_SSID,
            WIFI_AP_CHANNEL,
            wifi_dev.get_ip() or WIFI_AP_IP,
        )
    )
    print("IP addresses after AP:", sys.ip_address())
    return wifi_dev


def validate_configuration():
    """检查模型、网页、ROI、搜索覆盖范围和H_demo标定参数。"""
    if MODEL_WIDTH != MODEL_HEIGHT:
        raise RuntimeError("YOLOv5 model input must be square")
    if ROI_WIDTH > AI_WIDTH or ROI_HEIGHT > AI_HEIGHT:
        raise RuntimeError("ROI must fit inside the AI frame")
    if ROI_WIDTH * 9 != ROI_HEIGHT * 16:
        raise RuntimeError("ROI must keep the 16:9 training aspect ratio")

    sorted_rois = sorted(SEARCH_ROIS, key=lambda item: item[0])
    if sorted_rois[0][0] != 0:
        raise RuntimeError("Search ROIs must cover the left frame edge")
    if sorted_rois[-1][0] + ROI_WIDTH != AI_WIDTH:
        raise RuntimeError("Search ROIs must cover the right frame edge")

    covered_until = 0
    for roi_x, roi_y in sorted_rois:
        if roi_x < 0 or roi_y < 0:
            raise RuntimeError("Search ROI origin must not be negative")
        if roi_x > covered_until:
            raise RuntimeError("Search ROIs contain a horizontal gap")
        if roi_x + ROI_WIDTH > AI_WIDTH:
            raise RuntimeError("Search ROI exceeds AI frame width")
        if roi_y + ROI_HEIGHT > AI_HEIGHT:
            raise RuntimeError("Search ROI exceeds AI frame height")
        covered_until = max(covered_until, roi_x + ROI_WIDTH)

    validate_calibration(
        AXIS_START_PX,
        AXIS_END_PX,
        AXIS_START_CM,
        AXIS_END_CM,
        AI_WIDTH,
        AI_HEIGHT,
    )
    if MIN_BALL_BOX_PX < 1 or MAX_BALL_BOX_PX <= MIN_BALL_BOX_PX:
        raise RuntimeError("Invalid ball box size limits")
    if MAX_BALL_ASPECT_RATIO < 1.0:
        raise RuntimeError("MAX_BALL_ASPECT_RATIO must be >= 1")
    if MAX_AXIS_DISTANCE_PX <= 0.0:
        raise RuntimeError("MAX_AXIS_DISTANCE_PX must be positive")
    if AXIS_RATIO_MARGIN < 0.0:
        raise RuntimeError("AXIS_RATIO_MARGIN must not be negative")
    if WEBRTC_PORT < 1 or WEBRTC_PORT > 65535:
        raise RuntimeError("WEBRTC_PORT must be in 1..65535")
    if WEBRTC_SIGNALING_PORT < 1 or WEBRTC_SIGNALING_PORT > 65535:
        raise RuntimeError("WEBRTC_SIGNALING_PORT must be in 1..65535")
    if WEBRTC_BITRATE <= 0 or WEBRTC_GOP <= 0:
        raise RuntimeError("WebRTC bitrate and GOP must be positive")
    if UART_UNITS_PER_CM <= 0.0:
        raise RuntimeError("UART_UNITS_PER_CM must be positive")
    if UART_ERROR_MIN != -112 or UART_ERROR_MAX != 111:
        raise RuntimeError("UART error range must remain -112..111")
    if ZERO_STABLE_SAMPLE_COUNT < 5:
        raise RuntimeError("Zero selection requires at least 5 samples")
    if ZERO_MAX_POSITION_SPAN_CM > 0.30:
        raise RuntimeError("Zero stability span must not exceed 0.30 cm")


def resolve_model_path():
    """返回应用目录中的MUD路径，并检查相对CVI模型存在。"""
    app_dir = os.path.dirname(os.path.abspath(__file__))
    mud_path = os.path.join(app_dir, MODEL_MUD_NAME)
    model_path = os.path.join(app_dir, MODEL_FILE_NAME)

    if not os.path.isfile(mud_path):
        raise RuntimeError(
            "MUD model description not found: {}".format(mud_path)
        )
    if not os.path.isfile(model_path):
        raise RuntimeError(
            "CVI model file not found: {}".format(model_path)
        )
    return mud_path


def load_detector():
    """加载并校验model-310817单类别448x448 YOLOv5。"""
    model_path = resolve_model_path()
    detector = nn.YOLOv5(
        model=model_path,
        dual_buff=ENABLE_DUAL_BUFFER,
    )

    labels = detector.labels
    if len(labels) != 1 or labels[BALL_CLASS_ID] != "ball":
        raise RuntimeError(
            "Expected one 'ball' label, got: {}".format(labels)
        )
    if (
        detector.input_width() != MODEL_WIDTH
        or detector.input_height() != MODEL_HEIGHT
    ):
        raise RuntimeError(
            "Expected model input {}x{}, got {}x{}".format(
                MODEL_WIDTH,
                MODEL_HEIGHT,
                detector.input_width(),
                detector.input_height(),
            )
        )

    print("Detector model:", model_path)
    print(
        "Detector input: {}x{}, format={}, labels={}".format(
            detector.input_width(),
            detector.input_height(),
            detector.input_format(),
            labels,
        )
    )
    print(
        "Detector thresholds: conf={:.2f}, iou={:.2f}".format(
            CONFIDENCE_THRESHOLD,
            IOU_THRESHOLD,
        )
    )
    return detector


def create_camera_channels():
    """先创建WebRTC NV21主通道，再添加独立RGB AI副通道。"""
    web_cam = camera.Camera(
        WEBRTC_WIDTH,
        WEBRTC_HEIGHT,
        image.Format.FMT_YVU420SP,
        fps=WEBRTC_FPS,
        buff_num=WEBRTC_BUFFER_COUNT,
    )

    channel_count = web_cam.get_ch_nums()
    if channel_count < 2:
        web_cam.close()
        raise RuntimeError(
            "Camera reports {} channel(s); WebRTC+AI requires at least 2".format(
                channel_count
            )
        )

    try:
        ai_cam = web_cam.add_channel(
            AI_WIDTH,
            AI_HEIGHT,
            image.Format.FMT_RGB888,
            fps=AI_FPS,
            buff_num=AI_BUFFER_COUNT,
        )
    except Exception:
        web_cam.close()
        raise

    print("Camera channels supported:", channel_count)
    return web_cam, ai_cam


def create_webrtc_server(web_cam):
    """绑定NV21主通道并启动WebRTC硬件H.264服务器。"""
    server = webrtc.WebRTC(
        ip="",
        port=WEBRTC_PORT,
        bitrate=WEBRTC_BITRATE,
        gop=WEBRTC_GOP,
        signaling_ip="",
        signaling_port=WEBRTC_SIGNALING_PORT,
        http_server=True,
    )
    err.check_raise(
        server.bind_camera(web_cam),
        "WebRTC bind_camera failed",
    )
    err.check_raise(server.start(), "WebRTC start failed")
    return server


def create_uart_sender():
    """映射UART1并打开115200 8N1串口。"""
    err.check_raise(
        pinmap.set_pin_function(UART_RX_PIN, "UART1_RX"),
        "Failed to map {} to UART1_RX".format(UART_RX_PIN),
    )
    err.check_raise(
        pinmap.set_pin_function(UART_TX_PIN, "UART1_TX"),
        "Failed to map {} to UART1_TX".format(UART_TX_PIN),
    )
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def clamp(value, minimum, maximum):
    """把数值限制在闭区间内。"""
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def encode_tracking_packet(tracking, dx_uart):
    """按下位机协议编码5字节大端有符号偏差帧。"""
    if dx_uart < -32768 or dx_uart > 32767:
        raise ValueError("dx out of int16 range: {}".format(dx_uart))

    encoded_dx = dx_uart & 0xFFFF
    return bytes(
        (
            UART_FRAME_HEADER,
            0x01 if tracking else 0x00,
            (encoded_dx >> 8) & 0xFF,
            encoded_dx & 0xFF,
            UART_FRAME_TAIL,
        )
    )


def packet_to_hex(packet):
    """生成便于串口联调的HEX字符串。"""
    return " ".join("{:02X}".format(byte) for byte in packet)


def scale_uart_error_cm(error_cm):
    """Convert calibrated centimeters to the unchanged int16 UART field."""
    return clamp(
        int(round(float(error_cm) * UART_UNITS_PER_CM)),
        UART_ERROR_MIN,
        UART_ERROR_MAX,
    )


def make_control_output(
    detected,
    calibration_valid,
    position_cm,
    zero_manager,
):
    """Return safe tracking/error/UART values for one AI result."""
    if not detected or not calibration_valid or position_cm is None:
        return False, 0.0, 0
    error_cm = zero_manager.error_cm(position_cm)
    return True, error_cm, scale_uart_error_cm(error_cm)


def process_pending_zero_request(
    zero_manager,
    detected,
    position_cm,
    status,
):
    """Execute one web request in the AI loop, never in the HTTP thread."""
    request = claim_zero_request()
    if request is None:
        return

    if request["action"] == "reset":
        selected_zero_cm = zero_manager.reset_zero()
        print(
            "ZERO_RESET selected_zero_cm={:.2f}".format(
                selected_zero_cm
            )
        )
        complete_zero_request(
            request["request_id"],
            True,
            "已恢复中心零点",
            selected_zero_cm,
        )
        return

    success, message = zero_manager.try_set_current_as_zero(
        CALIBRATION_VALID,
        detected,
        position_cm,
        status,
    )
    selected_zero_cm = zero_manager.get_zero_cm()
    if success:
        print(
            (
                "ZERO_SET position_cm={:.2f} "
                "selected_zero_cm={:.2f}"
            ).format(
                position_cm,
                selected_zero_cm,
            )
        )
    else:
        print(
            "ZERO_SET_FAILED reason={} selected_zero_cm={:.2f}".format(
                message,
                selected_zero_cm,
            )
        )
    complete_zero_request(
        request["request_id"],
        success,
        message,
        selected_zero_cm,
    )


def send_tracking_packet(serial_dev, tracking, dx_uart):
    """发送跟踪状态，异常时允许视觉循环继续。"""
    packet = encode_tracking_packet(tracking, dx_uart)
    try:
        written = serial_dev.write(packet)
        if written != len(packet):
            print(
                "[WARN] UART short write: expected={}, actual={}".format(
                    len(packet),
                    written,
                )
            )
            return False, packet
        return True, packet
    except Exception as exc:
        print("[WARN] UART write failed:", exc)
        return False, packet


def read_camera_frame(cam):
    """只读取RGB AI副通道；偶发超时时短暂退避。"""
    try:
        return cam.read()
    except Exception as exc:
        print("[WARN] Camera read failed:", exc)
        time.sleep_ms(READ_RETRY_DELAY_MS)
        return None


def evaluate_candidate(candidate):
    """对全图候选执行框尺寸和标定轴线过滤，返回拒绝原因或None。"""
    width = float(candidate["bbox_w"])
    height = float(candidate["bbox_h"])
    short_side = min(width, height)
    long_side = max(width, height)

    if short_side < MIN_BALL_BOX_PX or long_side > MAX_BALL_BOX_PX:
        return "box_size"
    if short_side <= 0.0 or long_side / short_side > MAX_BALL_ASPECT_RATIO:
        return "box_aspect"

    if not CALIBRATION_VALID:
        return None

    _position_cm, axis_ratio, axis_distance = position_from_pixel(
        (candidate["x"], candidate["y"]),
        AXIS_START_PX,
        AXIS_END_PX,
        AXIS_START_CM,
        AXIS_END_CM,
    )
    if axis_distance > MAX_AXIS_DISTANCE_PX:
        return "axis_distance"
    if (
        axis_ratio < -AXIS_RATIO_MARGIN
        or axis_ratio > 1.0 + AXIS_RATIO_MARGIN
    ):
        return "axis_endpoint"
    return None


def select_best_valid_ball(objects, roi_x, roi_y):
    """
    映射并过滤全部ball候选，再选置信度最高者。

    返回：(最佳全图候选或None, 框过滤拒绝数, 轴线过滤拒绝数)。
    """
    best_candidate = None
    box_rejected = 0
    axis_rejected = 0

    for obj in objects:
        if obj.class_id != BALL_CLASS_ID:
            continue
        candidate = map_ball_to_global(obj, roi_x, roi_y)
        reason = evaluate_candidate(candidate)
        if reason is not None:
            if reason.startswith("box_"):
                box_rejected += 1
            else:
                axis_rejected += 1
            continue
        if (
            best_candidate is None
            or candidate["score"] > best_candidate["score"]
        ):
            best_candidate = candidate

    return best_candidate, box_rejected, axis_rejected


def map_ball_to_global(ball, roi_x, roi_y):
    """把ROI内检测框和中心映射回640x360 AI全图。"""
    bbox_x = roi_x + ball.x
    bbox_y = roi_y + ball.y
    center_x = clamp(
        bbox_x + ball.w * 0.5,
        0.0,
        float(AI_WIDTH - 1),
    )
    center_y = clamp(
        bbox_y + ball.h * 0.5,
        0.0,
        float(AI_HEIGHT - 1),
    )
    return {
        "x": center_x,
        "y": center_y,
        "bbox_x": bbox_x,
        "bbox_y": bbox_y,
        "bbox_w": ball.w,
        "bbox_h": ball.h,
        "score": ball.score,
    }


class RoiScheduler:
    """丢失时搜索左右窗口，检出后生成横向跟随ROI。"""

    def __init__(self):
        self.search_index = 0
        self.track_x = None
        self.track_y = None
        self.missed = 0
        self.jump_rejected_total = 0

    def is_tracking(self):
        return self.track_x is not None

    def current_roi(self):
        if not self.is_tracking():
            roi_x, roi_y = SEARCH_ROIS[self.search_index]
            return roi_x, roi_y, "search", self.search_index

        roi_x = clamp(
            int(round(self.track_x)) - ROI_WIDTH // 2,
            0,
            AI_WIDTH - ROI_WIDTH,
        )
        return roi_x, ROI_Y, "track", -1

    def accept(self, center_x, center_y):
        if self.is_tracking():
            dx = center_x - self.track_x
            dy = center_y - self.track_y
            if (
                dx * dx + dy * dy
                > TRACK_MAX_JUMP_PX * TRACK_MAX_JUMP_PX
            ):
                self.jump_rejected_total += 1
                return False

        self.track_x = center_x
        self.track_y = center_y
        self.missed = 0
        return True

    def miss(self):
        if not self.is_tracking():
            self.search_index = (
                self.search_index + 1
            ) % len(SEARCH_ROIS)
            return "search_miss"

        self.missed += 1
        if self.missed <= TRACK_MISS_LIMIT:
            return "track_miss"

        last_x = self.track_x
        search_centers = [
            roi_x + ROI_WIDTH // 2
            for roi_x, _roi_y in SEARCH_ROIS
        ]
        self.search_index = min(
            range(len(search_centers)),
            key=lambda index: abs(search_centers[index] - last_x),
        )
        self.track_x = None
        self.track_y = None
        self.missed = 0
        return "track_released"


def make_roi_metadata(roi_x, roi_y, mode, search_index, timestamp_ms):
    """保存送入模型的ROI信息，供dual_buff下一轮结果回映射。"""
    return {
        "x": roi_x,
        "y": roi_y,
        "mode": mode,
        "search_index": search_index,
        "timestamp_ms": timestamp_ms,
    }


def draw_calibration_axis(
    frame,
    axis_color,
    zero_color,
):
    """绘制H_demo标定轴、两端点和0cm位置。"""
    frame.draw_line(
        AXIS_START_PX[0],
        AXIS_START_PX[1],
        AXIS_END_PX[0],
        AXIS_END_PX[1],
        axis_color,
        2,
    )
    frame.draw_cross(
        AXIS_START_PX[0],
        AXIS_START_PX[1],
        axis_color,
        7,
        2,
    )
    frame.draw_cross(
        AXIS_END_PX[0],
        AXIS_END_PX[1],
        axis_color,
        7,
        2,
    )
    zero_ratio = (
        (0.0 - AXIS_START_CM)
        / (AXIS_END_CM - AXIS_START_CM)
    )
    zero_x, zero_y = axis_point(
        zero_ratio,
        AXIS_START_PX,
        AXIS_END_PX,
    )
    frame.draw_cross(
        int(zero_x),
        int(zero_y),
        zero_color,
        9,
        2,
    )


def draw_ai_overlay(
    frame,
    submitted_meta,
    result_meta,
    measurement,
    filtered_x,
    filtered_y,
    position_cm,
    projected_x,
    projected_y,
    selected_zero_cm,
    error_cm,
    dx_uart,
    status,
):
    """在本机AI调试画面绘制H_demo标识、目标和ROI范围。"""
    axis_color = image.Color.from_rgb(0, 220, 255)
    zero_color = image.Color.from_rgb(255, 220, 0)
    panel_color = image.Color.from_rgb(0, 0, 0)

    if CALIBRATION_VALID:
        draw_calibration_axis(frame, axis_color, zero_color)

    frame.draw_rect(
        submitted_meta["x"],
        submitted_meta["y"],
        ROI_WIDTH,
        ROI_HEIGHT,
        color=image.COLOR_BLUE,
        thickness=2,
    )
    if result_meta is not None:
        frame.draw_rect(
            result_meta["x"],
            result_meta["y"],
            ROI_WIDTH,
            ROI_HEIGHT,
            color=image.COLOR_YELLOW,
            thickness=1,
        )

    frame.draw_rect(
        0,
        0,
        AI_WIDTH,
        38,
        panel_color,
        -1,
    )

    if measurement is None:
        frame.draw_string(
            8,
            6,
            "BALL LOST {}".format(status),
            color=image.COLOR_RED,
            scale=1.2,
            thickness=2,
        )
        return

    frame.draw_rect(
        measurement["bbox_x"],
        measurement["bbox_y"],
        measurement["bbox_w"],
        measurement["bbox_h"],
        color=image.COLOR_RED,
        thickness=2,
    )
    frame.draw_cross(
        int(filtered_x),
        int(filtered_y),
        image.COLOR_GREEN,
        12,
        2,
    )
    if CALIBRATION_VALID:
        frame.draw_cross(
            int(projected_x),
            int(projected_y),
            zero_color,
            7,
            2,
        )
        status_text = "BALL {:+.2f}cm z={:+.2f} e={:+.2f} u={}".format(
            position_cm,
            selected_zero_cm,
            error_cm,
            dx_uart,
        )
    else:
        status_text = "BALL UNCALIBRATED uart=0"
    frame.draw_string(
        8,
        6,
        status_text,
        color=image.COLOR_WHITE,
        scale=1.2,
        thickness=2,
    )


def print_webrtc_urls(server):
    """打印WebRTC内置播放网页；iPad优先使用wlan0对应URL。"""
    urls = server.get_urls()
    if urls:
        print("WebRTC URLs (open wlan0/LAN URL on iPad):")
        for url in urls:
            print(" ", url)
    else:
        print("WebRTC URL:", server.get_url())


def print_record_urls():
    """打印自定义录像控制页；iPad优先使用wlan0对应URL。"""
    addresses = sys.ip_address()
    print("Browser recorder URLs (open wlan0/LAN URL on iPad):")
    if not addresses:
        print("  http://<MaixCAM-IP>:{}".format(RECORD_HTTP_PORT))
        return
    for interface_name, address in addresses.items():
        print(
            "  {}: http://{}:{}/".format(
                interface_name,
                address,
                RECORD_HTTP_PORT,
            )
        )


def main():
    """启动固定热点、WebRTC、浏览器录像页、ROI检测和UART发送。"""
    validate_configuration()
    print_runtime_info()
    detector = load_detector()

    web_cam = None
    ai_cam = None
    webrtc_server = None
    record_http_server = None
    record_http_thread = None
    serial_dev = None
    wifi_dev = None
    webrtc_started = False

    try:
        wifi_dev = create_wifi_ap()
        serial_dev = create_uart_sender()
        web_cam, ai_cam = create_camera_channels()

        # 所有权边界：必须先add_channel()，再绑定；绑定后不读取web_cam。
        webrtc_server = create_webrtc_server(web_cam)
        webrtc_started = True
        print_webrtc_urls(webrtc_server)
        record_http_server, record_http_thread = start_record_server()
        print_record_urls()

        disp = display.Display() if ENABLE_DISPLAY else None
        scheduler = RoiScheduler()
        position_filter = AdaptiveAlphaBetaFilter()
        zero_manager = SelectedZeroManager(
            SELECTED_ZERO_CM_DEFAULT,
            ZERO_STABLE_SAMPLE_COUNT,
            ZERO_MAX_POSITION_SPAN_CM,
        )
        update_zero_runtime_status(
            zero_manager.get_zero_cm(),
            CALIBRATION_VALID,
        )
        pending_roi_meta = None

        report_start_ms = time.ticks_ms()
        inference_count = 0
        detected_count = 0
        not_found_count = 0
        warmup_count = 0
        stale_result_count = 0
        search_frame_count = 0
        track_frame_count = 0
        box_rejected_count = 0
        axis_rejected_count = 0
        uart_sent_count = 0
        uart_failure_total = 0
        read_failure_total = 0

        while not app.need_exit():
            frame = read_camera_frame(ai_cam)
            if frame is None:
                read_failure_total += 1
                reason = scheduler.miss()
                status = "read_failure_{}".format(reason)
                position_filter.mark_missing(time.ticks_ms())
                zero_manager.record_result(
                    None,
                    CALIBRATION_VALID,
                    False,
                    status,
                )
                process_pending_zero_request(
                    zero_manager,
                    False,
                    None,
                    status,
                )
                update_zero_runtime_status(
                    zero_manager.get_zero_cm(),
                    CALIBRATION_VALID,
                )
                sent_ok, packet = send_tracking_packet(
                    serial_dev,
                    False,
                    0,
                )
                if sent_ok:
                    uart_sent_count += 1
                else:
                    uart_failure_total += 1
                if VERBOSE_RESULT_LOG:
                    print(
                        (
                            "BALL not_found reason=read_failure/{} "
                            "tracking=0 dx_uart=0 uart={}"
                        ).format(
                            reason,
                            packet_to_hex(packet),
                        )
                    )
                continue

            roi_x, roi_y, mode, search_index = scheduler.current_roi()
            submitted_meta = make_roi_metadata(
                roi_x,
                roi_y,
                mode,
                search_index,
                time.ticks_ms(),
            )
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

            if ENABLE_DUAL_BUFFER:
                result_meta = pending_roi_meta
                pending_roi_meta = submitted_meta
            else:
                result_meta = submitted_meta

            measurement = None
            filtered_x = 0.0
            filtered_y = 0.0
            position_cm = None
            projected_x = 0.0
            projected_y = 0.0
            selected_zero_cm = zero_manager.get_zero_cm()
            error_cm = 0.0
            dx_uart = 0
            status = "warmup"
            detected = False
            tracking = False

            if result_meta is None:
                warmup_count += 1
            else:
                inference_count += 1
                if result_meta["mode"] == "search":
                    search_frame_count += 1
                else:
                    track_frame_count += 1

                result_age_ms = (
                    time.ticks_ms() - result_meta["timestamp_ms"]
                )
                if result_age_ms > RESULT_MAX_AGE_MS:
                    stale_result_count += 1
                    status = "stale"
                    scheduler.miss()
                    position_filter.mark_missing(time.ticks_ms())
                else:
                    (
                        candidate,
                        frame_box_rejected,
                        frame_axis_rejected,
                    ) = select_best_valid_ball(
                        objects,
                        result_meta["x"],
                        result_meta["y"],
                    )
                    box_rejected_count += frame_box_rejected
                    axis_rejected_count += frame_axis_rejected
                    if candidate is None:
                        if frame_axis_rejected:
                            filter_reason = "axis_filter"
                        elif frame_box_rejected:
                            filter_reason = "box_filter"
                        else:
                            filter_reason = "miss"
                        miss_state = scheduler.miss()
                        status = "{}_{}".format(
                            filter_reason,
                            miss_state,
                        )
                        position_filter.mark_missing(time.ticks_ms())
                    else:
                        if scheduler.accept(
                            candidate["x"],
                            candidate["y"],
                        ):
                            measurement = candidate
                            filtered_x, filtered_y = (
                                position_filter.update(
                                    candidate["x"],
                                    candidate["y"],
                                    result_meta["timestamp_ms"],
                                )
                            )
                            (
                                position_cm,
                                axis_ratio,
                                _axis_distance,
                            ) = position_from_pixel(
                                (filtered_x, filtered_y),
                                AXIS_START_PX,
                                AXIS_END_PX,
                                AXIS_START_CM,
                                AXIS_END_CM,
                            )
                            projected_x, projected_y = axis_point(
                                axis_ratio,
                                AXIS_START_PX,
                                AXIS_END_PX,
                            )
                            detected_count += 1
                            detected = True
                            status = "detected"
                        else:
                            status = "jump"
                            scheduler.miss()
                            position_filter.mark_missing(
                                time.ticks_ms()
                            )

            zero_manager.record_result(
                position_cm,
                CALIBRATION_VALID,
                detected,
                status,
            )
            process_pending_zero_request(
                zero_manager,
                detected,
                position_cm,
                status,
            )
            selected_zero_cm = zero_manager.get_zero_cm()
            update_zero_runtime_status(
                selected_zero_cm,
                CALIBRATION_VALID,
            )

            tracking, error_cm, dx_uart = make_control_output(
                detected,
                CALIBRATION_VALID,
                position_cm,
                zero_manager,
            )

            if not detected and result_meta is not None:
                not_found_count += 1

            sent_ok, packet = send_tracking_packet(
                serial_dev,
                tracking,
                dx_uart if tracking else 0,
            )
            if sent_ok:
                uart_sent_count += 1
            else:
                uart_failure_total += 1

            if VERBOSE_RESULT_LOG:
                if detected:
                    position_text = (
                        "{:+.2f}".format(position_cm)
                        if CALIBRATION_VALID
                        else "UNCALIBRATED"
                    )
                    print(
                        (
                            "BALL x_full={:.1f} y_full={:.1f} "
                            "filtered=({:.1f},{:.1f}) position_cm={} "
                            "zero_cm={:+.2f} error_cm={:+.2f} "
                            "dx_uart={} score={:.3f} "
                            "result_mode={} result_roi=({},{},{},{}) "
                            "tracking={} uart={}"
                        ).format(
                            measurement["x"],
                            measurement["y"],
                            filtered_x,
                            filtered_y,
                            position_text,
                            selected_zero_cm,
                            error_cm,
                            dx_uart,
                            measurement["score"],
                            result_meta["mode"],
                            result_meta["x"],
                            result_meta["y"],
                            ROI_WIDTH,
                            ROI_HEIGHT,
                            1 if tracking else 0,
                            packet_to_hex(packet),
                        )
                    )
                else:
                    result_mode = (
                        "none"
                        if result_meta is None
                        else result_meta["mode"]
                    )
                    print(
                        (
                            "BALL not_found reason={} result_mode={} "
                            "submit_roi=({},{},{},{}) "
                            "tracking=0 dx_uart=0 uart={}"
                        ).format(
                            status,
                            result_mode,
                            submitted_meta["x"],
                            submitted_meta["y"],
                            ROI_WIDTH,
                            ROI_HEIGHT,
                            packet_to_hex(packet),
                        )
                    )

            if disp is not None:
                # 叠加标识仅用于MaixCAM/MaixVision本机画面。
                # WebRTC主通道保持完整1280x720原始画面，不读取也不回写。
                draw_ai_overlay(
                    frame,
                    submitted_meta,
                    result_meta,
                    measurement,
                    filtered_x,
                    filtered_y,
                    position_cm,
                    projected_x,
                    projected_y,
                    selected_zero_cm,
                    error_cm,
                    dx_uart,
                    status,
                )
                disp.show(frame)

            now_ms = time.ticks_ms()
            elapsed_ms = now_ms - report_start_ms
            if elapsed_ms >= STATS_INTERVAL_MS:
                inference_fps = (
                    inference_count * 1000.0 / elapsed_ms
                )
                if tracking:
                    print(
                        (
                            "STATS inference_fps={:.1f} "
                            "detected={}/{} tracking=1 "
                            "position_cm={:+.2f} zero_cm={:+.2f} "
                            "error_cm={:+.2f} dx_uart={} uart={}"
                        ).format(
                            inference_fps,
                            detected_count,
                            inference_count,
                            position_cm,
                            selected_zero_cm,
                            error_cm,
                            dx_uart,
                            packet_to_hex(packet),
                        )
                    )
                else:
                    print(
                        (
                            "STATS inference_fps={:.1f} "
                            "detected={}/{} tracking=0 "
                            "dx_uart=0 uart={}"
                        ).format(
                            inference_fps,
                            detected_count,
                            inference_count,
                            packet_to_hex(packet),
                        )
                    )
                report_start_ms = now_ms
                inference_count = 0
                detected_count = 0
                not_found_count = 0
                warmup_count = 0
                stale_result_count = 0
                search_frame_count = 0
                track_frame_count = 0
                box_rejected_count = 0
                axis_rejected_count = 0
                uart_sent_count = 0
    finally:
        stop_record_server(record_http_server, record_http_thread)
        if webrtc_server is not None and webrtc_started:
            stop_result = webrtc_server.stop()
            if stop_result != err.Err.ERR_NONE:
                print(
                    "[WARN] WebRTC stop failed:",
                    err.to_str(stop_result),
                )
        if ai_cam is not None:
            ai_cam.close()
        if web_cam is not None:
            web_cam.close()
        if serial_dev is not None:
            close_result = serial_dev.close()
            if close_result != err.Err.ERR_NONE:
                print(
                    "[WARN] UART close failed:",
                    err.to_str(close_result),
                )


if __name__ == "__main__":
    main()
