"""
model-310817 正式程序的标定与误检过滤参数。

先运行 calibrate.py。把它在 CALIBRATION_READY 后打印的整段参数发给 Codex，
再由 Codex 写回本文件。CALIBRATION_VALID=False 时，正式程序不会启用轴线过滤，
以免示例坐标误删真实钢球；框尺寸过滤仍会工作。
"""

# 640x360 AI 全图坐标，由最终安装状态下的 calibrate.py 实测。
AXIS_START_PX = (18, 144)
AXIS_END_PX = (606, 140)

# 实测物理坐标：左端 -11.5 cm，右端 +11.5 cm；保持原UART控制方向。
AXIS_START_CM = -11.5
AXIS_END_CM = 11.5
CALIBRATION_VALID = True

# 框尺寸过滤（单位：640x360 AI 全图像素）。
# 初值故意较宽；标定后按 calibrate.py 输出收紧。
MIN_BALL_BOX_PX = 14
MAX_BALL_BOX_PX = 67
MAX_BALL_ASPECT_RATIO = 1.80

# 轴线过滤：中心到标定轴线的最大垂直距离，以及允许越过端点的轴长比例。
# 仅在 CALIBRATION_VALID=True 时生效。
MAX_AXIS_DISTANCE_PX = 24.0
AXIS_RATIO_MARGIN = 0.08

# UART厘米偏差和进程内可选零点。
UART_UNITS_PER_CM = 8.0
SELECTED_ZERO_CM_DEFAULT = 0.0
ZERO_STABLE_SAMPLE_COUNT = 5
ZERO_MAX_POSITION_SPAN_CM = 0.30
