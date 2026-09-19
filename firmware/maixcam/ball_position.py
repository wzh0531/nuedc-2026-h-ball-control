"""Hardware-independent geometry and filtering for steel-ball tracking."""

import math


def project_onto_axis(point, axis_start, axis_end):
    """Return normalized position and perpendicular distance to an image axis."""
    px, py = point
    x0, y0 = axis_start
    x1, y1 = axis_end
    dx = x1 - x0
    dy = y1 - y0
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        raise ValueError("calibration endpoints must be different")

    relative_x = px - x0
    relative_y = py - y0
    ratio = (relative_x * dx + relative_y * dy) / float(length_squared)
    projected_x = x0 + ratio * dx
    projected_y = y0 + ratio * dy
    distance = math.hypot(px - projected_x, py - projected_y)
    return ratio, distance


def axis_point(ratio, axis_start, axis_end):
    """Return the image-space point at *ratio* along an axis."""
    x0, y0 = axis_start
    x1, y1 = axis_end
    return (
        x0 + ratio * (x1 - x0),
        y0 + ratio * (y1 - y0),
    )


def position_from_pixel(
    point,
    axis_start,
    axis_end,
    start_position_cm,
    end_position_cm,
):
    """Project an image point onto the beam and convert it to centimeters."""
    if start_position_cm == end_position_cm:
        raise ValueError("physical calibration positions must be different")
    ratio, distance = project_onto_axis(point, axis_start, axis_end)
    position_cm = start_position_cm + ratio * (
        end_position_cm - start_position_cm
    )
    return position_cm, ratio, distance


class AdaptiveAlphaBetaFilter:
    """Velocity-aware position filter with adaptive measurement gain."""

    def __init__(
        self,
        alpha_slow=0.20,
        alpha_fast=0.95,
        beta_slow=0.01,
        beta_fast=0.12,
        fast_error_px=10.0,
        reset_ms=250,
    ):
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.beta_slow = beta_slow
        self.beta_fast = beta_fast
        self.fast_error_px = fast_error_px
        self.reset_ms = reset_ms
        self.reset()

    def reset(self):
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = None

    def update(self, measured_x, measured_y, now_ms):
        if self.x is None:
            self.x = float(measured_x)
            self.y = float(measured_y)
            self.last_ms = now_ms
            return self.x, self.y

        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.005, min(0.100, dt))

        predicted_x = self.x + self.vx * dt
        predicted_y = self.y + self.vy * dt
        error_x = measured_x - predicted_x
        error_y = measured_y - predicted_y
        error = math.hypot(error_x, error_y)

        motion = min(1.0, error / self.fast_error_px) ** 2
        alpha = self.alpha_slow + (self.alpha_fast - self.alpha_slow) * motion
        beta = self.beta_slow + (self.beta_fast - self.beta_slow) * motion

        self.x = predicted_x + alpha * error_x
        self.y = predicted_y + alpha * error_y
        self.vx += beta * error_x / dt
        self.vy += beta * error_y / dt
        self.last_ms = now_ms
        return self.x, self.y

    def mark_missing(self, now_ms):
        if self.last_ms is not None and now_ms - self.last_ms >= self.reset_ms:
            self.reset()


def validate_calibration(
    axis_start, axis_end, start_position_cm, end_position_cm, width, height
):
    """Validate calibration against the detector image dimensions."""
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    for name, point in (("start", axis_start), ("end", axis_end)):
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise ValueError("{} calibration point is outside the image".format(name))
    position_from_pixel(
        axis_start,
        axis_start,
        axis_end,
        start_position_cm,
        end_position_cm,
    )


class SelectedZeroManager:
    """Manage a volatile user-selected zero from stable calibrated positions."""

    def __init__(
        self,
        default_zero_cm=0.0,
        stable_sample_count=5,
        max_position_span_cm=0.30,
    ):
        if stable_sample_count < 1:
            raise ValueError("stable_sample_count must be positive")
        if max_position_span_cm < 0.0:
            raise ValueError("max_position_span_cm must not be negative")

        self.default_zero_cm = float(default_zero_cm)
        self.stable_sample_count = int(stable_sample_count)
        self.max_position_span_cm = float(max_position_span_cm)
        self.selected_zero_cm = self.default_zero_cm
        self._recent_positions_cm = []

    def get_zero_cm(self):
        """Return the currently selected zero in centimeters."""
        return self.selected_zero_cm

    def reset_zero(self):
        """Restore the process-local default zero and return it."""
        self.selected_zero_cm = self.default_zero_cm
        return self.selected_zero_cm

    def error_cm(self, position_cm):
        """Return a calibrated position relative to the selected zero."""
        return float(position_cm) - self.selected_zero_cm

    def clear_samples(self):
        """Drop stability history after any invalid tracking result."""
        self._recent_positions_cm = []

    def record_result(
        self,
        position_cm,
        calibration_valid,
        tracking,
        status,
    ):
        """Record one consecutive valid position for later zero selection."""
        finite_position_cm = self._finite_position_or_none(position_cm)
        if (
            not calibration_valid
            or not tracking
            or self._status_error(status) is not None
            or finite_position_cm is None
        ):
            self.clear_samples()
            return False

        self._recent_positions_cm.append(finite_position_cm)
        if len(self._recent_positions_cm) > self.stable_sample_count:
            self._recent_positions_cm.pop(0)
        return True

    def try_set_current_as_zero(
        self,
        calibration_valid,
        tracking,
        position_cm,
        status,
    ):
        """Set zero only when calibration and recent tracking are safe."""
        if not calibration_valid:
            return False, "尚未完成标定"

        status_error = self._status_error(status)
        if status_error is not None:
            return False, status_error
        if not tracking:
            return False, "当前未检测到钢球"
        finite_position_cm = self._finite_position_or_none(position_cm)
        if finite_position_cm is None:
            return False, "当前位置无效"
        if len(self._recent_positions_cm) < self.stable_sample_count:
            return False, "有效位置样本不足{}帧".format(
                self.stable_sample_count
            )

        stability_window_cm = list(self._recent_positions_cm)
        stability_window_cm.append(finite_position_cm)
        position_span_cm = (
            max(stability_window_cm)
            - min(stability_window_cm)
        )
        if position_span_cm > self.max_position_span_cm:
            return (
                False,
                "位置不稳定：最近{}帧极差{:.2f}cm，允许≤{:.2f}cm".format(
                    self.stable_sample_count,
                    position_span_cm,
                    self.max_position_span_cm,
                ),
            )

        self.selected_zero_cm = finite_position_cm
        return True, "零点设置成功"

    @staticmethod
    def _finite_position_or_none(position_cm):
        if position_cm is None:
            return None
        try:
            value = float(position_cm)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value

    @classmethod
    def _status_error(cls, status):
        status_text = "" if status is None else str(status)
        if "axis_filter" in status_text:
            return "当前结果存在axis_filter错误"
        if "stale" in status_text:
            return "当前结果存在stale错误"
        if "jump" in status_text:
            return "当前结果存在jump错误"
        if "miss" in status_text:
            return "当前结果存在miss错误"
        return None
