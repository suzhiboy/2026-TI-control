"""
K230 YOLO steel-ball position sender for the 2026 TI H problem.

UART wiring:
K230 GPIO11 TX -> MSPM0G3507 PA1 RX
K230 GPIO12 RX -> MSPM0G3507 PA0 TX
GND -> GND
"""

import gc
import os
import time

try:
    import ujson
except ImportError:
    import json as ujson

try:
    import aicube
except ImportError:
    aicube = None

try:
    import nncase_runtime as nn
except ImportError:
    nn = None

try:
    import ulab.numpy as np
except ImportError:
    np = None

try:
    import image
except ImportError:
    image = None

try:
    from machine import FPIOA, UART
except ImportError:
    FPIOA = None
    UART = None

try:
    from machine import WDT
except ImportError:
    WDT = None

try:
    from media.sensor import *
except ImportError:
    pass

try:
    from media.display import *
except ImportError:
    Display = None

try:
    from media.media import *
except ImportError:
    MediaManager = None


DEPLOY_ROOT_PATH = "/sdcard/mp_deployment_source/"
FALLBACK_DEPLOY_ROOT_PATH = "/data/data/mp_deployment_source/"
DEPLOY_CONFIG_NAME = "deploy_config.json"
MODEL_PATH = DEPLOY_ROOT_PATH + "best_AnchorBaseDet_can2_5_n_20260730052458.kmodel"
FALLBACK_MODEL_PATH = FALLBACK_DEPLOY_ROOT_PATH + "best_AnchorBaseDet_can2_5_n_20260730052458.kmodel"
LABELS = ["ball"]
MODEL_INPUT_SIZE = [320, 320]

DISPLAY_MODE = "lcd"  # lcd or hdmi
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
AI_FRAME_WIDTH = 1088
AI_FRAME_HEIGHT = 720
RGB888P_SIZE = [AI_FRAME_WIDTH, AI_FRAME_HEIGHT]

# Competition mode: keep UART real-time, reduce IDE/display load.
# Enable display/prints only while tuning the camera and rod endpoints.
USE_ROI_REFINE = False
DRAW_RESULT = False
SHOW_IMAGE = True
DEBUG_FRAME_STATUS = False
PRINT_MEMORY = False
STOP_AFTER_SECONDS = 0
UART_FRAME_MODE = "compact_csv"  # compact_csv or legacy_ball
RAW_RESULT_PRINT_EVERY_N_FRAMES = 0  # Anchor debug; set to 0 for competition.
YOLO_DEBUG_MAX_BOXES = 3

# Tune these points after camera placement is fixed.
# Values below are YOLO box-center coordinates measured on the K230 terminal.
CALIBRATION_MODE = "linear"  # Endpoint-only projection along the rod line.
ROD_LEFT_PX = (31, 219)
ROD_RIGHT_PX = (745, 229)
ROD_CENTER_PX = (
    (ROD_LEFT_PX[0] + ROD_RIGHT_PX[0]) // 2,
    (ROD_LEFT_PX[1] + ROD_RIGHT_PX[1]) // 2,
)
ROD_LEFT_MM = -120
ROD_CENTER_MM = 0
ROD_RIGHT_MM = 120
ROD_DX = float(ROD_RIGHT_PX[0] - ROD_LEFT_PX[0])
ROD_DY = float(ROD_RIGHT_PX[1] - ROD_LEFT_PX[1])
ROD_LEN2 = ROD_DX * ROD_DX + ROD_DY * ROD_DY
ROD_MM_RANGE = ROD_RIGHT_MM - ROD_LEFT_MM
if ROD_LEN2 > 0.0:
    ROD_CENTER_T = (
        (float(ROD_CENTER_PX[0] - ROD_LEFT_PX[0]) * ROD_DX)
        + (float(ROD_CENTER_PX[1] - ROD_LEFT_PX[1]) * ROD_DY)
    ) / ROD_LEN2
else:
    ROD_CENTER_T = 0.5

# Keep ROD_*_PX in DISPLAY_WIDTH x DISPLAY_HEIGHT control coordinates printed
# by K230_FRAME. AnchorBaseDet AI coordinates are scaled into this space first.

# ROI refinement is optional because it costs CPU and requires box/image coordinates
# to be in the same space. Leave it off unless this has been verified on the IDE.
ROI_EXPAND_RATIO = 0.35
ROI_MIN_MARGIN_PX = 8
ROI_SCAN_STEP = 4
BRIGHT_PERCENTILE_DIVISOR = 5
BRIGHT_MIN_DELTA = 18
MIN_BRIGHT_PIXELS = 6
FILTER_ALPHA = 0.85
MEDIAN_WINDOW = 1

RESULT_SCORE_INDEX = "auto"  # auto, 1, or 2. Official docs commonly use boxes,scores,classes.
BOX_FORMAT = "auto"  # auto, xywh, or xyxy.
CONF_THRESH = 0.50
NMS_THRESH = 0.50
MAX_BOXES_NUM = 5
MIN_BOX_SIZE_PX = 4
MAX_BOX_SIZE_PX = 180
MAX_BOX_ASPECT_RATIO = 2.4
ROD_MAX_PERP_PX = 55 # 0 disables rod-distance hard gate.
ROD_ENDPOINT_MARGIN_PX = 10
MERGE_IOU_THRESH = 0.30

MAX_JUMP_MM = 120
INIT_CONFIRM_FRAMES = 1
INIT_CONFIRM_SPREAD_MM = 15
ALLOW_JUMP_RELOCK = True
REINIT_AFTER_LOST_FRAMES = 5
JUMP_CONFIRM_FRAMES = 2
JUMP_CONFIRM_SPREAD_MM = 20
LOST_FRAME_LIMIT = 1
SEND_EVERY_N_FRAMES = 1
TERMINAL_PRINT_EVERY_N_FRAMES = 30
LOST_SEND_EVERY_N_FRAMES = 1
MEMORY_PRINT_EVERY_N_FRAMES = 30
GC_EVERY_N_FRAMES = 30
MAIN_LOOP_SLEEP_MS = 0
ENABLE_WATCHDOG = True
WATCHDOG_ID = 0
WATCHDOG_FALLBACK_IDS = (1, 2)
WATCHDOG_TIMEOUT_SECONDS = 5
PIPELINE_HEARTBEAT_EVERY_N_FRAMES = 0


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def resolve_model_path():
    if file_exists(MODEL_PATH):
        return MODEL_PATH
    if file_exists(FALLBACK_MODEL_PATH):
        return FALLBACK_MODEL_PATH
    raise RuntimeError("kmodel not found on /sdcard or /data/data")


def resolve_deploy_root_path():
    sd_config = DEPLOY_ROOT_PATH + DEPLOY_CONFIG_NAME
    if file_exists(sd_config):
        return DEPLOY_ROOT_PATH

    fallback_config = FALLBACK_DEPLOY_ROOT_PATH + DEPLOY_CONFIG_NAME
    if file_exists(fallback_config):
        return FALLBACK_DEPLOY_ROOT_PATH

    raise RuntimeError("deploy_config.json not found in mp_deployment_source")


def read_deploy_config(config_path):
    with open(config_path, "r") as json_file:
        return ujson.load(json_file)


def runtime_ready():
    return aicube is not None and nn is not None and np is not None and image is not None


def get_display_config(display):
    if display == "hdmi":
        return "hdmi", [1920, 1080], Sensor(width=1920, height=1080)
    if display == "lcd2_4":
        return "st7701", [640, 480], Sensor(width=1280, height=960)
    if display == "lcd3_5":
        return "st7701", [800, 480], Sensor(width=1920, height=1080)
    raise ValueError("DISPLAY must be hdmi, lcd3_5, or lcd2_4")


def setup_uart2():
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def uart_send(uart, text):
    uart.write(text)


def clamp_position_mm(x_mm):
    if x_mm < ROD_LEFT_MM:
        return ROD_LEFT_MM
    if x_mm > ROD_RIGHT_MM:
        return ROD_RIGHT_MM
    return x_mm


def round_int(value):
    if value is None:
        return 0
    value = float(value)
    if value >= 0.0:
        return int(value + 0.5)
    return int(value - 0.5)


def quality_from_conf(conf):
    quality = round_int(float(conf) * 100.0)
    if quality < 0:
        return 0
    if quality > 100:
        return 100
    return quality


def linear_mm_from_t(t):
    return float(ROD_LEFT_MM) + float(t) * float(ROD_MM_RANGE)


def calibrated_mm_from_t(t):
    t = float(t)
    if CALIBRATION_MODE != "three_point":
        return linear_mm_from_t(t)

    center_t = float(ROD_CENTER_T)
    denom = center_t * center_t - center_t
    if denom > -0.0001 and denom < 0.0001:
        return linear_mm_from_t(t)

    right_delta = float(ROD_RIGHT_MM - ROD_LEFT_MM)
    center_delta = float(ROD_CENTER_MM - ROD_LEFT_MM)
    a = (center_delta - right_delta * center_t) / denom
    b = right_delta - a
    return float(ROD_LEFT_MM) + a * t * t + b * t


def rod_endpoint_margin_t():
    if ROD_LEN2 <= 0.0:
        return 0.0
    return float(ROD_ENDPOINT_MARGIN_PX) / float(ROD_LEN2 ** 0.5)


def rod_t_in_extended_range(raw_t):
    margin_t = rod_endpoint_margin_t()
    return raw_t >= -margin_t and raw_t <= 1.0 + margin_t


def project_point_to_rod_raw_info(px, py):
    if ROD_LEN2 <= 0.0:
        return 0.0, 0.0, 0.0, 0.0

    vx = float(px) - float(ROD_LEFT_PX[0])
    vy = float(py) - float(ROD_LEFT_PX[1])
    raw_t = (vx * ROD_DX + vy * ROD_DY) / ROD_LEN2
    t = raw_t
    if t < 0.0:
        t = 0.0
    if t > 1.0:
        t = 1.0

    closest_x = float(ROD_LEFT_PX[0]) + t * ROD_DX
    closest_y = float(ROD_LEFT_PX[1]) + t * ROD_DY
    err_x = float(px) - closest_x
    err_y = float(py) - closest_y
    distance2 = err_x * err_x + err_y * err_y
    x_mm = calibrated_mm_from_t(t)
    return raw_t, t, x_mm, distance2


def project_point_to_rod_info(px, py):
    if ROD_LEN2 <= 0.0:
        return 0.0, 0.0, 0.0

    _, t, x_mm, distance2 = project_point_to_rod_raw_info(px, py)
    return t, x_mm, distance2


def project_point_to_rod_mm(px, py):
    _, x_mm, _ = project_point_to_rod_info(px, py)
    return clamp_position_mm(round_int(x_mm))


def ai_to_control_x(x):
    return float(x) * float(DISPLAY_WIDTH) / float(AI_FRAME_WIDTH)


def ai_to_control_y(y):
    return float(y) * float(DISPLAY_HEIGHT) / float(AI_FRAME_HEIGHT)


def aicube_box_center_control(det_box):
    if list_len(det_box) < 6:
        return None, None
    x1 = float(det_box[2])
    y1 = float(det_box[3])
    x2 = float(det_box[4])
    y2 = float(det_box[5])
    cx_ai = (x1 + x2) / 2.0
    cy_ai = (y1 + y2) / 2.0
    return ai_to_control_x(cx_ai), ai_to_control_y(cy_ai)


def aicube_box_area_ai(det_box):
    if list_len(det_box) < 6:
        return 0.0
    w = float(det_box[4]) - float(det_box[2])
    h = float(det_box[5]) - float(det_box[3])
    if w <= 0.0 or h <= 0.0:
        return 0.0
    return w * h


def candidate_from_aicube_box(det_box):
    if list_len(det_box) < 6:
        return None
    conf = float(det_box[1])
    if conf < CONF_THRESH:
        return None

    cx, cy = aicube_box_center_control(det_box)
    if cx is None:
        return None
    raw_t, _, raw_x_mm_float, distance2 = project_point_to_rod_raw_info(cx, cy)
    if not rod_t_in_extended_range(raw_t):
        return None
    if ROD_MAX_PERP_PX > 0:
        max_distance2 = float(ROD_MAX_PERP_PX * ROD_MAX_PERP_PX)
        if distance2 > max_distance2:
            return None

    raw_x_mm = clamp_position_mm(round_int(raw_x_mm_float))
    quality = quality_from_conf(conf)
    area = aicube_box_area_ai(det_box)
    return det_box, cx, cy, conf, raw_x_mm, quality, distance2, area


def aicube_candidate_reject_reason(det_boxes):
    box_count = list_len(det_boxes)
    if box_count <= 0:
        return "no_box"

    bad_box_count = 0
    low_conf_count = 0
    outside_range_count = 0
    off_rod_count = 0
    for i in range(box_count):
        det_box = det_boxes[i]
        if list_len(det_box) < 6:
            bad_box_count += 1
            continue

        conf = float(det_box[1])
        if conf < CONF_THRESH:
            low_conf_count += 1
            continue

        cx, cy = aicube_box_center_control(det_box)
        if cx is None:
            bad_box_count += 1
            continue

        raw_t, _, _, distance2 = project_point_to_rod_raw_info(cx, cy)
        if not rod_t_in_extended_range(raw_t):
            outside_range_count += 1
            continue

        if ROD_MAX_PERP_PX > 0:
            max_distance2 = float(ROD_MAX_PERP_PX * ROD_MAX_PERP_PX)
            if distance2 > max_distance2:
                off_rod_count += 1
                continue

        return "candidate_unselected"

    if outside_range_count > 0:
        return "outside_rod_range"
    if off_rod_count > 0:
        return "off_rod"
    if low_conf_count > 0:
        return "low_conf"
    if bad_box_count > 0:
        return "bad_box"
    return "no_candidate"


def aicube_candidate_score(candidate, last_x_mm=None):
    _, _, _, conf, raw_x_mm, _, distance2, area = candidate
    candidate_score = float(area) * 0.001 + float(conf) * 10.0
    if ROD_MAX_PERP_PX > 0:
        candidate_score -= (
            distance2 / float(ROD_MAX_PERP_PX * ROD_MAX_PERP_PX)
        ) * 2.0
    if last_x_mm is not None and MAX_JUMP_MM > 0:
        jump_ratio = abs(raw_x_mm - int(last_x_mm)) / float(MAX_JUMP_MM)
        if jump_ratio > 3.0:
            jump_ratio = 3.0
        candidate_score -= jump_ratio * 0.8
    return candidate_score


def candidate_is_trackable(candidate, last_x_mm):
    if last_x_mm is None or MAX_JUMP_MM <= 0:
        return False
    _, _, _, _, raw_x_mm, _, _, _ = candidate
    return abs(raw_x_mm - int(last_x_mm)) <= MAX_JUMP_MM


def decode_aicube_det_boxes(det_boxes, last_x_mm=None):
    if det_boxes is None or list_len(det_boxes) <= 0:
        return None, None, None, 0.0, None, 0

    best_trackable = None
    best_trackable_score = -999999.0
    best_fallback = None
    best_fallback_score = -999999.0
    count = list_len(det_boxes)
    for i in range(count):
        candidate = candidate_from_aicube_box(det_boxes[i])
        if candidate is None:
            continue

        candidate_score = aicube_candidate_score(candidate, last_x_mm)
        if candidate_is_trackable(candidate, last_x_mm):
            if candidate_score > best_trackable_score:
                best_trackable_score = candidate_score
                best_trackable = candidate
        elif candidate_score > best_fallback_score:
            best_fallback_score = candidate_score
            best_fallback = candidate

    best_candidate = best_trackable
    if best_candidate is None:
        best_candidate = best_fallback

    if best_candidate is None:
        return None, None, None, 0.0, None, 0

    box, cx, cy, conf, raw_x_mm, quality, _, _ = best_candidate
    return box, cx, cy, conf, raw_x_mm, quality


def build_anchor_debug_lines(frame_id, det_boxes):
    lines = []
    box_count = list_len(det_boxes)
    lines.append("K230_ANCHOR frame={} boxes={}".format(frame_id, box_count))
    if box_count <= 0:
        return lines

    count = box_count
    if count > YOLO_DEBUG_MAX_BOXES:
        count = YOLO_DEBUG_MAX_BOXES
    for i in range(count):
        det_box = det_boxes[i]
        if list_len(det_box) < 6:
            lines.append(
                "K230_ANCHOR_BOX frame={} i={} invalid={}".format(
                    frame_id,
                    i,
                    text_limited(det_box, 96),
                )
            )
            continue

        cx, cy = aicube_box_center_control(det_box)
        raw_x_mm = project_point_to_rod_mm(cx, cy)
        _, _, distance2 = project_point_to_rod_info(cx, cy)
        lines.append(
            "K230_ANCHOR_BOX frame={} i={} cls={} score={} cx={} cy={} raw_mm={} area={} perp_px={} raw={}".format(
                frame_id,
                i,
                int(det_box[0]),
                text_limited(det_box[1], 24),
                round_int(cx),
                round_int(cy),
                raw_x_mm,
                round_int(aicube_box_area_ai(det_box)),
                round_int(distance2 ** 0.5),
                text_limited(det_box, 128),
            )
        )

    return lines


def print_anchor_debug(frame_id, det_boxes):
    if RAW_RESULT_PRINT_EVERY_N_FRAMES <= 0:
        return
    if frame_id % RAW_RESULT_PRINT_EVERY_N_FRAMES != 0:
        return
    try:
        lines = build_anchor_debug_lines(frame_id, det_boxes)
        for line in lines:
            print(line)
    except Exception as e:
        print("K230_ANCHOR_PRINT_FAIL frame={} err={}".format(frame_id, e))


def merge_overlap_boxes(boxes):
    if boxes is None or list_len(boxes) <= 1:
        return boxes

    merged = []
    used = [False] * list_len(boxes)
    for i in range(list_len(boxes)):
        if used[i]:
            continue
        if list_len(boxes[i]) < 6:
            used[i] = True
            continue

        best_cls = boxes[i][0]
        best_conf = boxes[i][1]
        x1 = boxes[i][2]
        y1 = boxes[i][3]
        x2 = boxes[i][4]
        y2 = boxes[i][5]
        used[i] = True

        changed = True
        while changed:
            changed = False
            for j in range(list_len(boxes)):
                if used[j] or list_len(boxes[j]) < 6:
                    continue

                bx1 = boxes[j][2]
                by1 = boxes[j][3]
                bx2 = boxes[j][4]
                by2 = boxes[j][5]
                ix1 = x1 if x1 > bx1 else bx1
                iy1 = y1 if y1 > by1 else by1
                ix2 = x2 if x2 < bx2 else bx2
                iy2 = y2 if y2 < by2 else by2
                iw = ix2 - ix1
                ih = iy2 - iy1
                if iw < 0:
                    iw = 0
                if ih < 0:
                    ih = 0
                inter = iw * ih
                area_a = (x2 - x1) * (y2 - y1)
                area_b = (bx2 - bx1) * (by2 - by1)
                union = area_a + area_b - inter
                if union <= 0:
                    iou = 0
                else:
                    iou = inter / union

                if iou > MERGE_IOU_THRESH:
                    x1 = x1 if x1 < bx1 else bx1
                    y1 = y1 if y1 < by1 else by1
                    x2 = x2 if x2 > bx2 else bx2
                    y2 = y2 if y2 > by2 else by2
                    if boxes[j][1] > best_conf:
                        best_conf = boxes[j][1]
                        best_cls = boxes[j][0]
                    used[j] = True
                    changed = True

        merged.append([best_cls, best_conf, x1, y1, x2, y2])

    return merged


def post_process_anchor_outputs(results, deploy_conf, frame_size):
    model_type = deploy_conf["model_type"]
    kmodel_frame_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    nms_option = deploy_conf["nms_option"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    strides = [8, 16, 32]

    if model_type == "AnchorBaseDet":
        anchors = (
            deploy_conf["anchors"][0]
            + deploy_conf["anchors"][1]
            + deploy_conf["anchors"][2]
        )
        det_boxes = aicube.anchorbasedet_post_process(
            results[0],
            results[1],
            results[2],
            kmodel_frame_size,
            frame_size,
            strides,
            num_classes,
            confidence_threshold,
            nms_threshold,
            anchors,
            nms_option,
        )
    elif model_type == "GFLDet":
        det_boxes = aicube.gfldet_post_process(
            results[0],
            results[1],
            results[2],
            kmodel_frame_size,
            frame_size,
            strides,
            num_classes,
            confidence_threshold,
            nms_threshold,
            nms_option,
        )
    else:
        det_boxes = aicube.anchorfreedet_post_process(
            results[0],
            results[1],
            results[2],
            kmodel_frame_size,
            frame_size,
            strides,
            num_classes,
            confidence_threshold,
            nms_threshold,
            nms_option,
        )

    return merge_overlap_boxes(det_boxes)


def normalize_box_as(box, box_format):
    if list_len(box) < 4:
        return None

    x0 = float(box[0])
    y0 = float(box[1])
    if box_format == "xyxy":
        w = float(box[2]) - x0
        h = float(box[3]) - y0
    else:
        w = float(box[2])
        h = float(box[3])

    if w <= 0.0 or h <= 0.0:
        return None
    return x0, y0, w, h


def box_normalizations(box):
    if BOX_FORMAT == "xywh":
        normalized = normalize_box_as(box, "xywh")
        if normalized is None:
            return []
        return [normalized]
    if BOX_FORMAT == "xyxy":
        normalized = normalize_box_as(box, "xyxy")
        if normalized is None:
            return []
        return [normalized]

    normalizations = []
    normalized = normalize_box_as(box, "xywh")
    if normalized is None:
        pass
    else:
        normalizations.append(normalized)

    normalized = normalize_box_as(box, "xyxy")
    if normalized is None:
        pass
    else:
        normalizations.append(normalized)

    return normalizations


def normalized_box_center(normalized):
    x, y, w, h = normalized
    return x + w / 2.0, y + h / 2.0


def normalized_geometry_plausible(normalized):
    if normalized is None:
        return False

    _, _, w, h = normalized
    if w < MIN_BOX_SIZE_PX or h < MIN_BOX_SIZE_PX:
        return False
    if w > MAX_BOX_SIZE_PX or h > MAX_BOX_SIZE_PX:
        return False

    if w > h:
        ratio = w / h
    else:
        ratio = h / w
    return ratio <= MAX_BOX_ASPECT_RATIO


def normalize_box(box):
    normalizations = box_normalizations(box)
    for normalized in normalizations:
        if normalized_geometry_plausible(normalized):
            return normalized
    if list_len(normalizations) > 0:
        return normalizations[0]
    return None


def yolo_box_center(box):
    normalized = normalize_box(box)
    if normalized is None:
        return 0.0, 0.0
    return normalized_box_center(normalized)


def is_box_geometry_plausible(box):
    normalizations = box_normalizations(box)
    for normalized in normalizations:
        if normalized_geometry_plausible(normalized):
            return True
    return False


def candidate_from_box(box, conf):
    normalizations = box_normalizations(box)
    best_candidate = None
    best_distance2 = None
    for normalized in normalizations:
        if not normalized_geometry_plausible(normalized):
            continue

        cx, cy = normalized_box_center(normalized)
        raw_t, _, raw_x_mm_float, distance2 = project_point_to_rod_raw_info(cx, cy)
        if not rod_t_in_extended_range(raw_t):
            continue
        if ROD_MAX_PERP_PX > 0:
            max_distance2 = float(ROD_MAX_PERP_PX * ROD_MAX_PERP_PX)
            if distance2 > max_distance2:
                continue

        raw_x_mm = clamp_position_mm(round_int(raw_x_mm_float))
        quality = quality_from_conf(conf)
        candidate = box, cx, cy, float(conf), raw_x_mm, quality, distance2
        if best_candidate is None or distance2 < best_distance2:
            best_candidate = candidate
            best_distance2 = distance2

    return best_candidate


class PositionTracker:
    def __init__(
        self,
        alpha=FILTER_ALPHA,
        max_jump_mm=MAX_JUMP_MM,
        median_window=MEDIAN_WINDOW,
        init_confirm_frames=INIT_CONFIRM_FRAMES,
        init_confirm_spread_mm=INIT_CONFIRM_SPREAD_MM,
        allow_jump_relock=ALLOW_JUMP_RELOCK,
        reinit_after_lost_frames=REINIT_AFTER_LOST_FRAMES,
        jump_confirm_frames=JUMP_CONFIRM_FRAMES,
        jump_confirm_spread_mm=JUMP_CONFIRM_SPREAD_MM,
    ):
        self.alpha = float(alpha)
        self.max_jump_mm = int(max_jump_mm)
        self.median_window = int(median_window)
        self.init_confirm_frames = int(init_confirm_frames)
        self.init_confirm_spread_mm = int(init_confirm_spread_mm)
        self.allow_jump_relock = bool(allow_jump_relock)
        self.reinit_after_lost_frames = int(reinit_after_lost_frames)
        self.jump_confirm_frames = int(jump_confirm_frames)
        self.jump_confirm_spread_mm = int(jump_confirm_spread_mm)
        self.samples = []
        self.last_raw_x_mm = None
        self.last_x_mm = None
        self.filtered_x_mm = None
        self.lost_frames = 0
        self.pending_init_count = 0
        self.pending_init_min = None
        self.pending_init_max = None
        self.pending_init_last = None
        self.pending_jump_count = 0
        self.pending_jump_min = None
        self.pending_jump_max = None
        self.pending_jump_last = None
        self.last_reject_reason = "reset"

    def reset(self):
        self.samples = []
        self.last_raw_x_mm = None
        self.last_x_mm = None
        self.filtered_x_mm = None
        self.lost_frames = 0
        self._clear_pending_init()
        self._clear_pending_jump()
        self.last_reject_reason = "reset"

    def mark_lost(self, reason="lost"):
        self.lost_frames += 1
        self.last_reject_reason = reason

    def _clear_pending_init(self):
        self.pending_init_count = 0
        self.pending_init_min = None
        self.pending_init_max = None
        self.pending_init_last = None

    def _clear_pending_jump(self):
        self.pending_jump_count = 0
        self.pending_jump_min = None
        self.pending_jump_max = None
        self.pending_jump_last = None

    def _pending_init_confirmed(self, raw_x_mm):
        if self.init_confirm_frames <= 1:
            return True

        if (
            self.pending_init_last is None
            or abs(raw_x_mm - self.pending_init_last) > self.init_confirm_spread_mm
        ):
            self.pending_init_count = 1
            self.pending_init_min = raw_x_mm
            self.pending_init_max = raw_x_mm
            self.pending_init_last = raw_x_mm
            return False

        self.pending_init_count += 1
        if raw_x_mm < self.pending_init_min:
            self.pending_init_min = raw_x_mm
        if raw_x_mm > self.pending_init_max:
            self.pending_init_max = raw_x_mm
        self.pending_init_last = raw_x_mm

        spread = self.pending_init_max - self.pending_init_min
        return (
            self.pending_init_count >= self.init_confirm_frames
            and spread <= self.init_confirm_spread_mm
        )

    def _pending_jump_confirmed(self, raw_x_mm):
        if self.jump_confirm_frames <= 1:
            return True

        if (
            self.pending_jump_last is None
            or abs(raw_x_mm - self.pending_jump_last) > self.jump_confirm_spread_mm
        ):
            self.pending_jump_count = 1
            self.pending_jump_min = raw_x_mm
            self.pending_jump_max = raw_x_mm
            self.pending_jump_last = raw_x_mm
            return False

        self.pending_jump_count += 1
        if raw_x_mm < self.pending_jump_min:
            self.pending_jump_min = raw_x_mm
        if raw_x_mm > self.pending_jump_max:
            self.pending_jump_max = raw_x_mm
        self.pending_jump_last = raw_x_mm

        spread = self.pending_jump_max - self.pending_jump_min
        return (
            self.pending_jump_count >= self.jump_confirm_frames
            and spread <= self.jump_confirm_spread_mm
        )

    def _median_sample(self):
        if len(self.samples) <= 0:
            return None
        ordered = list(self.samples)
        ordered.sort()
        return ordered[len(ordered) // 2]

    def _accept_raw(self, raw_x_mm, reset_filter=False):
        if reset_filter:
            self.samples = []

        self.samples.append(raw_x_mm)
        while len(self.samples) > self.median_window:
            self.samples.pop(0)

        median_x_mm = self._median_sample()
        if reset_filter or self.filtered_x_mm is None:
            self.filtered_x_mm = float(median_x_mm)
        else:
            self.filtered_x_mm = (
                self.filtered_x_mm * (1.0 - self.alpha)
                + float(median_x_mm) * self.alpha
            )

        self.last_raw_x_mm = raw_x_mm
        self.last_x_mm = clamp_position_mm(round_int(self.filtered_x_mm))
        self.lost_frames = 0
        self._clear_pending_init()
        self._clear_pending_jump()
        self.last_reject_reason = "ok"
        return True, self.last_x_mm

    def _clear_track_for_reinit(self):
        self.samples = []
        self.last_raw_x_mm = None
        self.last_x_mm = None
        self.filtered_x_mm = None
        self._clear_pending_init()
        self._clear_pending_jump()

    def _try_initial_accept(self, raw_x_mm):
        if not self._pending_init_confirmed(raw_x_mm):
            self.mark_lost(
                "init_confirm:{}/{}".format(
                    self.pending_init_count,
                    self.init_confirm_frames,
                )
            )
            return False, None
        return self._accept_raw(raw_x_mm, reset_filter=True)

    def update(self, raw_x_mm):
        if raw_x_mm is None:
            self._clear_pending_init()
            self._clear_pending_jump()
            self.mark_lost("no_candidate")
            return False, None

        raw_x_mm = clamp_position_mm(int(raw_x_mm))
        if self.last_raw_x_mm is None:
            return self._try_initial_accept(raw_x_mm)

        if (
            self.max_jump_mm > 0
            and abs(raw_x_mm - self.last_raw_x_mm) > self.max_jump_mm
        ):
            if (
                self.reinit_after_lost_frames > 0
                and self.lost_frames >= self.reinit_after_lost_frames
            ):
                self._clear_track_for_reinit()
                return self._try_initial_accept(raw_x_mm)

            jump_confirmed = self._pending_jump_confirmed(raw_x_mm)
            if self.allow_jump_relock and jump_confirmed:
                return self._accept_raw(raw_x_mm, reset_filter=True)

            self.mark_lost("jump_reject:{}".format(self.pending_jump_count))
            return False, None

        return self._accept_raw(raw_x_mm)


def clamp_int(value, low, high):
    value = int(value)
    if value < low:
        return low
    if value > high:
        return high
    return value


def get_image_hw(img):
    try:
        return int(img.shape[1]), int(img.shape[2])
    except Exception:
        return RGB888P_SIZE[1], RGB888P_SIZE[0]


def get_gray_from_rgb888p(img, x, y):
    try:
        r = int(img[0][y][x])
        g = int(img[1][y][x])
        b = int(img[2][y][x])
    except Exception:
        try:
            r = int(img[0, y, x])
            g = int(img[1, y, x])
            b = int(img[2, y, x])
        except Exception:
            return None
    return (r * 30 + g * 59 + b * 11) // 100


def roi_from_box(box, img_w, img_h):
    normalized = normalize_box(box)
    if normalized is None:
        return 0, 0, img_w - 1, img_h - 1

    x, y, w, h = normalized
    margin = int((w if w > h else h) * ROI_EXPAND_RATIO)
    if margin < ROI_MIN_MARGIN_PX:
        margin = ROI_MIN_MARGIN_PX

    x0 = clamp_int(x - margin, 0, img_w - 1)
    y0 = clamp_int(y - margin, 0, img_h - 1)
    x1 = clamp_int(x + w + margin, 0, img_w - 1)
    y1 = clamp_int(y + h + margin, 0, img_h - 1)
    return x0, y0, x1, y1


def refine_ball_center_in_roi(img, box):
    img_h, img_w = get_image_hw(img)
    x0, y0, x1, y1 = roi_from_box(box, img_w, img_h)

    min_gray = 255
    max_gray = 0
    for y in range(y0, y1 + 1, ROI_SCAN_STEP):
        for x in range(x0, x1 + 1, ROI_SCAN_STEP):
            gray = get_gray_from_rgb888p(img, x, y)
            if gray is None:
                continue
            if gray < min_gray:
                min_gray = gray
            if gray > max_gray:
                max_gray = gray

    if max_gray <= min_gray:
        return yolo_box_center(box), False

    delta = max_gray - min_gray
    threshold = max_gray - delta // BRIGHT_PERCENTILE_DIVISOR
    if threshold < min_gray + BRIGHT_MIN_DELTA:
        threshold = min_gray + BRIGHT_MIN_DELTA

    weight_sum = 0
    x_sum = 0
    y_sum = 0
    bright_count = 0
    for y in range(y0, y1 + 1, ROI_SCAN_STEP):
        for x in range(x0, x1 + 1, ROI_SCAN_STEP):
            gray = get_gray_from_rgb888p(img, x, y)
            if gray is None or gray < threshold:
                continue
            weight = gray - threshold + 1
            weight_sum += weight
            x_sum += x * weight
            y_sum += y * weight
            bright_count += 1

    if bright_count < MIN_BRIGHT_PIXELS or weight_sum <= 0:
        return yolo_box_center(box), False

    return (float(x_sum) / float(weight_sum), float(y_sum) / float(weight_sum)), True


def list_len(value):
    try:
        return len(value)
    except Exception:
        return 0


def safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def is_integer_like(value):
    value_int = int(value)
    diff = float(value) - float(value_int)
    if diff < 0.0:
        diff = -diff
    return diff < 0.0001


def score_list_rank(values, count):
    if values is None or list_len(values) < count:
        return -1

    rank = 0
    for i in range(count):
        value = safe_float(values[i])
        if value is None:
            return -1
        if value < 0.0 or value > 1.5:
            return -1
        if value >= CONF_THRESH:
            rank += 4
        if value > 0.0 and value <= 1.0:
            rank += 1
        if not is_integer_like(value):
            rank += 3
    return rank


def select_score_index(result, count):
    if list_len(result) < 3:
        return 0
    if RESULT_SCORE_INDEX == 1:
        return 1
    if RESULT_SCORE_INDEX == 2:
        return 2

    rank1 = score_list_rank(result[1], count)
    rank2 = score_list_rank(result[2], count)
    if rank1 < 0 and rank2 < 0:
        return 0
    if rank2 > rank1:
        return 2
    return 1


def select_score_list(result, count):
    score_index = select_score_index(result, count)
    if score_index <= 0:
        return None
    return result[score_index]


def text_limited(value, limit=96):
    try:
        text = str(value)
    except Exception as e:
        text = "<str_fail:{}>".format(e)

    if len(text) > limit:
        return text[:limit] + "..."
    return text


def first_items_text(value, max_items=3, limit=96):
    count = list_len(value)
    if count <= 0:
        return "[]"

    text = "["
    shown = count if count < max_items else max_items
    for i in range(shown):
        if i > 0:
            text += ";"
        try:
            item = value[i]
        except Exception as e:
            item = "<get_fail:{}>".format(e)
        text += text_limited(item, limit)
    if count > shown:
        text += ";..."
    text += "]"
    return text


def format_normalized_box_debug(box, box_format):
    normalized = normalize_box_as(box, box_format)
    if normalized is None:
        return "fmt={} invalid_norm".format(box_format)

    x, y, w, h = normalized
    geom = 1 if normalized_geometry_plausible(normalized) else 0
    cx, cy = normalized_box_center(normalized)
    _, raw_x_mm_float, distance2 = project_point_to_rod_info(cx, cy)
    raw_x_mm = clamp_position_mm(round_int(raw_x_mm_float))
    perp_px = round_int(distance2 ** 0.5)
    if ROD_MAX_PERP_PX > 0:
        near_rod = 1 if distance2 <= float(ROD_MAX_PERP_PX * ROD_MAX_PERP_PX) else 0
    else:
        near_rod = -1
    return (
        "fmt={} x={} y={} w={} h={} cx={} cy={} raw_mm={} perp_px={} geom={} near_rod={}".format(
            box_format,
            round_int(x),
            round_int(y),
            round_int(w),
            round_int(h),
            round_int(cx),
            round_int(cy),
            raw_x_mm,
            perp_px,
            geom,
            near_rod,
        )
    )


def build_yolo_debug_lines(frame_id, result):
    lines = []
    result_len = list_len(result)
    lines.append(
        "K230_YOLO_RAW frame={} result_len={} raw={}".format(
            frame_id,
            result_len,
            text_limited(result, 220),
        )
    )

    for i in range(result_len):
        try:
            item = result[i]
        except Exception as e:
            lines.append(
                "K230_YOLO_PART frame={} idx={} get_fail={}".format(frame_id, i, e)
            )
            continue
        lines.append(
            "K230_YOLO_PART frame={} idx={} len={} first={}".format(
                frame_id,
                i,
                list_len(item),
                first_items_text(item, YOLO_DEBUG_MAX_BOXES, 96),
            )
        )

    if result is None or result_len < 3:
        return lines

    boxes = result[0]
    box_count = list_len(boxes)
    rank1 = score_list_rank(result[1], box_count)
    rank2 = score_list_rank(result[2], box_count)
    chosen = select_score_index(result, box_count)
    lines.append(
        "K230_YOLO_SCORE frame={} boxes={} rank1={} rank2={} chosen={} conf_thresh={:.2f}".format(
            frame_id,
            box_count,
            rank1,
            rank2,
            chosen,
            CONF_THRESH,
        )
    )

    if box_count <= 0:
        return lines

    scores = select_score_list(result, box_count)
    count = box_count
    if scores is not None and list_len(scores) < count:
        count = list_len(scores)
    if count > YOLO_DEBUG_MAX_BOXES:
        count = YOLO_DEBUG_MAX_BOXES

    for i in range(count):
        try:
            box = boxes[i]
        except Exception as e:
            lines.append("K230_YOLO_BOX frame={} i={} box_get_fail={}".format(frame_id, i, e))
            continue

        score = None
        if scores is not None:
            try:
                score = scores[i]
            except Exception:
                score = None

        lines.append(
            "K230_YOLO_BOX frame={} i={} score={} raw_box={}".format(
                frame_id,
                i,
                text_limited(score, 32),
                text_limited(box, 128),
            )
        )
        lines.append(
            "K230_YOLO_BOX_FMT frame={} i={} {}".format(
                frame_id,
                i,
                format_normalized_box_debug(box, "xywh"),
            )
        )
        lines.append(
            "K230_YOLO_BOX_FMT frame={} i={} {}".format(
                frame_id,
                i,
                format_normalized_box_debug(box, "xyxy"),
            )
        )

    return lines


def decode_canmv_yolo_result(result, last_x_mm=None):
    if result is None or list_len(result) < 3:
        return None, None, None, 0.0, None, 0

    boxes = result[0]
    box_count = list_len(boxes)
    if box_count <= 0:
        return None, None, None, 0.0, None, 0

    scores = select_score_list(result, box_count)
    score_count = list_len(scores)
    if score_count <= 0:
        return None, None, None, 0.0, None, 0

    best_candidate = None
    best_score = -999.0
    count = box_count if box_count < score_count else score_count
    for i in range(count):
        score = float(scores[i])
        if score < CONF_THRESH:
            continue

        candidate = candidate_from_box(boxes[i], score)
        if candidate is None:
            continue

        _, _, _, conf, raw_x_mm, _, distance2 = candidate
        candidate_score = conf
        if ROD_MAX_PERP_PX > 0:
            candidate_score -= (
                distance2 / float(ROD_MAX_PERP_PX * ROD_MAX_PERP_PX)
            ) * 0.20
        if last_x_mm is not None and MAX_JUMP_MM > 0:
            jump_ratio = abs(raw_x_mm - int(last_x_mm)) / float(MAX_JUMP_MM)
            if jump_ratio > 3.0:
                jump_ratio = 3.0
            candidate_score -= jump_ratio * 0.06

        if candidate_score > best_score:
            best_score = candidate_score
            best_candidate = candidate

    if best_candidate is None:
        return None, None, None, 0.0, None, 0

    box, cx, cy, conf, raw_x_mm, quality, _ = best_candidate
    return box, cx, cy, conf, raw_x_mm, quality


def make_ball_frame(seq, valid, x_mm, raw_x_mm, cx, cy, quality, fps):
    if not valid:
        x_mm = 0
        raw_x_mm = 0
        cx = 0
        cy = 0
        quality = 0

    return "B,{},{},{},{},{},{},{},{}\n".format(
        int(seq),
        1 if valid else 0,
        int(x_mm),
        int(raw_x_mm),
        round_int(cx),
        round_int(cy),
        int(quality),
        round_int(fps),
    )


def send_position(
    uart,
    seq,
    valid,
    x_mm,
    raw_x_mm,
    cx,
    cy,
    conf,
    quality,
    fps,
    log=False,
):
    if UART_FRAME_MODE == "legacy_ball":
        if valid:
            uart_send(uart, "BALL,{},{},{:.2f},{}\n".format(seq, x_mm, conf, int(fps)))
        else:
            uart_send(uart, "BALL_LOST,{}\n".format(seq))
    else:
        uart_send(
            uart,
            make_ball_frame(seq, valid, x_mm, raw_x_mm, cx, cy, quality, fps),
        )

    if log:
        if valid:
            print(
                "K230_BALL seq={} x_mm={} raw={} cx={} cy={} q={} fps={} sent=1".format(
                    seq,
                    x_mm,
                    raw_x_mm,
                    round_int(cx),
                    round_int(cy),
                    quality,
                    int(fps),
                )
            )
        else:
            print("K230_BALL_LOST seq={} fps={} sent=1".format(seq, int(fps)))


def send_ball(uart, seq, x_mm, conf, fps, log=False):
    send_position(
        uart,
        seq,
        True,
        x_mm,
        x_mm,
        0,
        0,
        conf,
        quality_from_conf(conf),
        fps,
        log,
    )


def send_lost(uart, seq, log=False):
    send_position(uart, seq, False, None, None, None, None, 0.0, 0, 0, log)


def safe_mem_free():
    try:
        return gc.mem_free()
    except Exception:
        return -1


def setup_watchdog():
    if not ENABLE_WATCHDOG or WDT is None:
        print("K230_WDT disabled unavailable")
        return None

    watchdog_ids = (WATCHDOG_ID,) + WATCHDOG_FALLBACK_IDS
    last_error = "unknown"
    for watchdog_id in watchdog_ids:
        try:
            watchdog = WDT(watchdog_id, WATCHDOG_TIMEOUT_SECONDS)
            print(
                "K230_WDT enabled id={} timeout_s={}".format(
                    watchdog_id,
                    WATCHDOG_TIMEOUT_SECONDS,
                )
            )
            return watchdog
        except Exception as e:
            last_error = e

        try:
            watchdog = WDT(watchdog_id, timeout=WATCHDOG_TIMEOUT_SECONDS)
            print(
                "K230_WDT enabled id={} timeout_s={}".format(
                    watchdog_id,
                    WATCHDOG_TIMEOUT_SECONDS,
                )
            )
            return watchdog
        except Exception as e:
            last_error = e

    print("K230_WDT disabled ids={} err={}".format(watchdog_ids, last_error))
    return None


def feed_watchdog(watchdog):
    if watchdog is None:
        return
    try:
        watchdog.feed()
    except Exception as e:
        print("K230_WDT feed_fail err={}".format(e))


def print_pipeline_heartbeat(frame_id, stage, fps):
    if PIPELINE_HEARTBEAT_EVERY_N_FRAMES <= 0:
        return
    if frame_id % PIPELINE_HEARTBEAT_EVERY_N_FRAMES != 0:
        return
    print(
        "K230_HEARTBEAT frame={} stage={} fps={} heap={}".format(
            frame_id,
            stage,
            round_int(fps),
            safe_mem_free(),
        )
    )


def print_frame_status(
    frame_id,
    fps,
    seen,
    cx,
    cy,
    raw_x_mm,
    x_mm,
    conf,
    quality,
    lost_frames,
    sent,
    refined,
    reject_reason="",
):
    if not DEBUG_FRAME_STATUS:
        return
    if frame_id % TERMINAL_PRINT_EVERY_N_FRAMES != 0:
        return

    if seen:
        print(
            "K230_FRAME id={} fps={} seen=1 cx={} cy={} raw={} x_mm={} conf={:.2f} q={} refined={} lost={} sent={}".format(
                frame_id,
                int(fps),
                round_int(cx),
                round_int(cy),
                raw_x_mm,
                x_mm,
                conf,
                quality,
                refined,
                lost_frames,
                sent,
            )
        )
    else:
        if reject_reason is None or reject_reason == "":
            reject_reason = "unknown"
        print(
            "K230_FRAME id={} fps={} seen=0 cand_cx={} cand_cy={} cand_raw={} conf={:.2f} q={} lost={} sent={} reason={}".format(
                frame_id,
                int(fps),
                round_int(cx),
                round_int(cy),
                raw_x_mm,
                conf,
                quality,
                lost_frames,
                sent,
                reject_reason,
            )
        )


def print_memory_status(frame_id):
    if not PRINT_MEMORY:
        return
    if frame_id % MEMORY_PRINT_EVERY_N_FRAMES != 0:
        return
    print("K230_MEM frame={} free={}".format(frame_id, safe_mem_free()))


def print_raw_yolo_result(frame_id, result):
    if RAW_RESULT_PRINT_EVERY_N_FRAMES <= 0:
        return
    if frame_id % RAW_RESULT_PRINT_EVERY_N_FRAMES != 0:
        return
    try:
        lines = build_yolo_debug_lines(frame_id, result)
        for line in lines:
            print(line)
    except Exception as e:
        print("K230_RAW_RESULT_PRINT_FAIL frame={} err={}".format(frame_id, e))


def safe_deinit_uart(uart):
    if uart is None:
        return
    try:
        uart.deinit()
    except Exception:
        pass


def safe_deinit_yolo(yolo):
    if yolo is None:
        return
    try:
        yolo.deinit()
    except Exception as e:
        print("K230_YOLO_DEINIT_FAIL:", e)


def safe_destroy_pipeline(pl):
    if pl is None:
        return
    try:
        pl.destroy()
    except Exception as e:
        print("K230_PIPELINE_DESTROY_FAIL:", e)


def sleep_ms(ms):
    if ms <= 0:
        return
    try:
        time.sleep_ms(ms)
    except Exception:
        time.sleep(float(ms) / 1000.0)


def setup_sensor_and_display():
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    if SHOW_IMAGE:
        sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
        sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)

    sensor.set_framesize(
        width=AI_FRAME_WIDTH,
        height=AI_FRAME_HEIGHT,
        chn=CAM_CHN_ID_2,
    )
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    if SHOW_IMAGE:
        sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
        if DISPLAY_MODE == "lcd":
            Display.init(Display.ST7701, to_ide=False)
        else:
            Display.init(Display.LT9611, to_ide=True)

    return sensor


def setup_anchor_runtime(deploy_conf, deploy_root):
    kmodel_name = deploy_conf["kmodel_path"]
    kmodel_frame_size = deploy_conf["img_size"]
    width = kmodel_frame_size[0]
    height = kmodel_frame_size[1]

    ratio_w = float(width) / float(AI_FRAME_WIDTH)
    ratio_h = float(height) / float(AI_FRAME_HEIGHT)
    ratio = ratio_w if ratio_w < ratio_h else ratio_h
    new_w = int(ratio * AI_FRAME_WIDTH)
    new_h = int(ratio * AI_FRAME_HEIGHT)
    dw = float(width - new_w) / 2.0
    dh = float(height - new_h) / 2.0
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))

    kpu = nn.kpu()
    ai2d = nn.ai2d()
    kpu.load_kmodel(deploy_root + kmodel_name)
    ai2d.set_dtype(
        nn.ai2d_format.NCHW_FMT,
        nn.ai2d_format.NCHW_FMT,
        np.uint8,
        np.uint8,
    )
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1, 3, AI_FRAME_HEIGHT, AI_FRAME_WIDTH],
        [1, 3, height, width],
    )
    data = np.ones((1, 3, height, width), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)
    return kpu, ai2d_builder, ai2d_output_tensor


def get_kpu_outputs(kpu):
    results = []
    for i in range(kpu.outputs_size()):
        out_data = kpu.get_output_tensor(i)
        result = out_data.to_numpy()
        result = result.reshape(
            (result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3])
        )
        del out_data
        results.append(result)
    return results


def draw_anchor_results(osd_img, selected_box, cx, cy, x_mm, fps):
    if not SHOW_IMAGE or osd_img is None:
        return

    osd_img.clear()

    if DRAW_RESULT:
        osd_img.draw_line(
            ROD_LEFT_PX[0],
            ROD_LEFT_PX[1],
            ROD_RIGHT_PX[0],
            ROD_RIGHT_PX[1],
            color=(255, 255, 0),
            thickness=2,
        )

        if selected_box is not None and list_len(selected_box) >= 6:
            x1 = ai_to_control_x(selected_box[2])
            y1 = ai_to_control_y(selected_box[3])
            x2 = ai_to_control_x(selected_box[4])
            y2 = ai_to_control_y(selected_box[5])
            osd_img.draw_rectangle(
                int(x1),
                int(y1),
                int(x2 - x1),
                int(y2 - y1),
                color=(0, 255, 0),
                thickness=2,
            )

        if cx is not None and cy is not None and x_mm is not None:
            osd_img.draw_circle(int(cx), int(cy), 4, color=(255, 0, 0), fill=True)
            osd_img.draw_string_advanced(
                2, 30, 24, "cx={}, cy={}".format(round_int(cx), round_int(cy)),
                color=(0, 255, 0),
            )
            osd_img.draw_string_advanced(
                2, 56, 24, "x={} mm".format(x_mm), color=(255, 0, 0)
            )
        else:
            osd_img.draw_string_advanced(2, 30, 24, "BALL LOST", color=(255, 0, 0))

        osd_img.draw_string_advanced(
            2, 2, 24, "FPS=" + str(int(fps)), color=(0, 255, 255)
        )
    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)


def safe_stop_sensor(sensor):
    if sensor is None:
        return
    try:
        sensor.stop()
    except Exception:
        pass


def safe_deinit_display():
    if not SHOW_IMAGE:
        return
    try:
        Display.deinit()
    except Exception:
        pass


def safe_deinit_media():
    try:
        MediaManager.deinit()
    except Exception:
        pass


def safe_exitpoint_sleep():
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except Exception:
        pass


def main():
    uart = None
    sensor = None
    osd_img = None
    watchdog = None
    ai2d_input_tensor = None
    ai2d_output_tensor = None
    seq = 0
    tracker = PositionTracker()

    try:
        if not runtime_ready():
            raise RuntimeError("K230 AnchorBaseDet runtime modules are not available")

        deploy_root = resolve_deploy_root_path()
        deploy_conf = read_deploy_config(deploy_root + DEPLOY_CONFIG_NAME)

        global LABELS, CONF_THRESH, NMS_THRESH, MODEL_INPUT_SIZE
        LABELS = deploy_conf["categories"]
        CONF_THRESH = deploy_conf["confidence_threshold"]
        NMS_THRESH = deploy_conf["nms_threshold"]
        MODEL_INPUT_SIZE = deploy_conf["img_size"]

        uart = setup_uart2()
        print(
            "K230_START anchor_uart baud=115200 mode={} conf={:.2f} draw={} show={}".format(
                UART_FRAME_MODE,
                CONF_THRESH,
                int(DRAW_RESULT),
                int(SHOW_IMAGE),
            )
        )
        print(
            "K230_CAL left_px={} center_px={} right_px={} mm=[{},{},{}] max_perp_px={} endpoint_margin={} max_jump_mm={} init_confirm={} init_spread={} allow_jump_relock={} reinit_lost={} jump_confirm={} jump_spread={}".format(
                ROD_LEFT_PX,
                ROD_CENTER_PX,
                ROD_RIGHT_PX,
                ROD_LEFT_MM,
                ROD_CENTER_MM,
                ROD_RIGHT_MM,
                ROD_MAX_PERP_PX,
                ROD_ENDPOINT_MARGIN_PX,
                MAX_JUMP_MM,
                INIT_CONFIRM_FRAMES,
                INIT_CONFIRM_SPREAD_MM,
                int(ALLOW_JUMP_RELOCK),
                REINIT_AFTER_LOST_FRAMES,
                JUMP_CONFIRM_FRAMES,
                JUMP_CONFIRM_SPREAD_MM,
            )
        )
        print(
            "K230_MODEL root={} model={} input={} ai_frame={} display={}".format(
                deploy_root,
                deploy_conf["kmodel_path"],
                MODEL_INPUT_SIZE,
                [AI_FRAME_WIDTH, AI_FRAME_HEIGHT],
                [DISPLAY_WIDTH, DISPLAY_HEIGHT],
            )
        )

        kpu, ai2d_builder, ai2d_output_tensor = setup_anchor_runtime(
            deploy_conf,
            deploy_root,
        )
        sensor = setup_sensor_and_display()
        if SHOW_IMAGE:
            osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
        MediaManager.init()
        sensor.run()
        watchdog = setup_watchdog()

        clock = time.clock()
        frame_id = 0
        start_ms = time.ticks_ms()
        frame_size = [AI_FRAME_WIDTH, AI_FRAME_HEIGHT]

        while True:
            os.exitpoint()
            if STOP_AFTER_SECONDS > 0:
                elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
                if elapsed_ms >= STOP_AFTER_SECONDS * 1000:
                    print("K230_STOP_BY_TIMER seconds={}".format(STOP_AFTER_SECONDS))
                    break

            rgb888p_img = None
            ai2d_input = None
            ai2d_input_tensor = None
            results = None
            det_boxes = None
            stage = "start"
            try:
                clock.tick()
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
                frame_id += 1
                stage = "snapshot"
                print_pipeline_heartbeat(frame_id, stage, clock.fps())

                sent = 0
                seen = False
                refined = 0
                x_mm = None
                raw_x_mm = None
                cx = None
                cy = None
                conf = 0.0
                quality = 0
                reject_reason = "none"

                if rgb888p_img.format() == image.RGBP888:
                    ai2d_input = rgb888p_img.to_numpy_ref()
                    ai2d_input_tensor = nn.from_numpy(ai2d_input)
                    ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                    stage = "ai2d"
                    print_pipeline_heartbeat(frame_id, stage, clock.fps())
                    kpu.set_input_tensor(0, ai2d_output_tensor)
                    kpu.run()
                    stage = "kpu"
                    print_pipeline_heartbeat(frame_id, stage, clock.fps())
                    results = get_kpu_outputs(kpu)
                    det_boxes = post_process_anchor_outputs(
                        results,
                        deploy_conf,
                        frame_size,
                    )
                    stage = "post"
                    print_pipeline_heartbeat(frame_id, stage, clock.fps())

                box, cx, cy, conf, raw_x_mm, quality = decode_aicube_det_boxes(
                    det_boxes,
                    tracker.last_raw_x_mm,
                )
                print_anchor_debug(frame_id, det_boxes)

                if box is not None:
                    accepted, x_mm = tracker.update(raw_x_mm)
                    reject_reason = tracker.last_reject_reason
                    if accepted:
                        seen = True
                        if frame_id % SEND_EVERY_N_FRAMES == 0:
                            seq = (seq + 1) & 0xFFFF
                            send_position(
                                uart,
                                seq,
                                True,
                                x_mm,
                                raw_x_mm,
                                cx,
                                cy,
                                conf,
                                quality,
                                clock.fps(),
                            )
                            sent = 1
                    else:
                        x_mm = None
                else:
                    tracker.update(None)
                    reject_reason = aicube_candidate_reject_reason(det_boxes)

                if (
                    tracker.lost_frames >= LOST_FRAME_LIMIT
                    and frame_id % LOST_SEND_EVERY_N_FRAMES == 0
                ):
                    seq = (seq + 1) & 0xFFFF
                    send_position(
                        uart,
                        seq,
                        False,
                        None,
                        None,
                        None,
                        None,
                        0.0,
                        0,
                        clock.fps(),
                    )
                    sent = 1
                stage = "uart"
                print_pipeline_heartbeat(frame_id, stage, clock.fps())

                draw_anchor_results(
                    osd_img,
                    box if seen else None,
                    cx if seen else None,
                    cy if seen else None,
                    x_mm,
                    clock.fps(),
                )
                stage = "display"
                print_pipeline_heartbeat(frame_id, stage, clock.fps())

                print_frame_status(
                    frame_id,
                    clock.fps(),
                    seen,
                    cx,
                    cy,
                    raw_x_mm,
                    x_mm,
                    conf,
                    quality,
                    tracker.lost_frames,
                    sent,
                    refined,
                    reject_reason,
                )
                print_memory_status(frame_id)
                stage = "complete"
                print_pipeline_heartbeat(frame_id, stage, clock.fps())
            finally:
                det_boxes = None
                results = None
                ai2d_input_tensor = None
                ai2d_input = None
                rgb888p_img = None

            feed_watchdog(watchdog)

            if frame_id % GC_EVERY_N_FRAMES == 0:
                gc.collect()
            sleep_ms(MAIN_LOOP_SLEEP_MS)

    except KeyboardInterrupt:
        print("K230_STOP_BY_IDE")

    except Exception as e:
        print("K230_ERROR:", e)

    finally:
        print("K230_RELEASE")
        safe_exitpoint_sleep()
        safe_stop_sensor(sensor)
        safe_deinit_display()
        safe_deinit_media()
        safe_deinit_uart(uart)
        try:
            del ai2d_input_tensor
        except Exception:
            pass
        try:
            del ai2d_output_tensor
        except Exception:
            pass
        gc.collect()
        if nn is not None:
            try:
                nn.shrink_memory_pool()
            except Exception:
                pass


if __name__ == "__main__":
    main()
