#!/usr/bin/env python3
# coding=utf-8

"""
Detect a calibration square with automatic color learning.

The old version only detected a hard-coded red block.  This version keeps the
public detect_red_block() entry point, but the detector is now designed for a
square marker of any visible color.  The user enters the physical side length
at startup, and the detector uses that size plus the depth value to reject
objects whose apparent pixel size is unreasonable.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


DEFAULT_SQUARE_SIZE_M = 0.03


@dataclass
class SquareDetection:
    contour: np.ndarray
    center_px: tuple[int, int]
    depth_m: float
    camera_xyz_m: tuple[float, float, float]
    rect_points: np.ndarray
    side_px: float
    expected_side_px: float
    score: float
    mean_hsv: np.ndarray


def ask_square_size_m(default_size_m: float = DEFAULT_SQUARE_SIZE_M) -> float:
    """Ask for the calibration square side length in meters."""
    try:
        import tkinter as tk
        from tkinter import simpledialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        value = simpledialog.askfloat(
            "Calibration square",
            "Square side length in centimeters:",
            minvalue=0.1,
            initialvalue=default_size_m * 100.0,
        )
        root.destroy()
        if value is not None:
            return float(value) / 100.0
    except Exception:
        pass

    while True:
        raw_value = input(f"Square side length in centimeters [{default_size_m * 100:.1f}]: ").strip()
        if not raw_value:
            return default_size_m
        try:
            value_cm = float(raw_value)
            if value_cm > 0:
                return value_cm / 100.0
        except ValueError:
            pass
        print("Please enter a positive number.")


def median_depth(depth_frame, cx: int, cy: int, radius: int = 3) -> float:
    """Use a small neighborhood to reduce single-pixel depth noise."""
    values = []
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    for y in range(max(0, cy - radius), min(height, cy + radius + 1)):
        for x in range(max(0, cx - radius), min(width, cx + radius + 1)):
            depth = depth_frame.get_distance(x, y)
            if depth > 0:
                values.append(depth)
    if not values:
        return 0.0
    return float(np.median(values))


def build_learned_color_mask(hsv: np.ndarray, learned_hsv: np.ndarray | None) -> np.ndarray | None:
    if learned_hsv is None:
        return None

    hue, saturation, value = [int(v) for v in learned_hsv]
    if saturation >= 35:
        hue_delta = 15
        lower_sv = np.array([0, max(30, saturation - 80), max(30, value - 100)], dtype=np.uint8)
        upper_sv = np.array([179, 255, 255], dtype=np.uint8)

        lower_hue = hue - hue_delta
        upper_hue = hue + hue_delta
        if lower_hue < 0:
            mask1 = cv2.inRange(hsv, np.array([0, lower_sv[1], lower_sv[2]], dtype=np.uint8), np.array([upper_hue, upper_sv[1], upper_sv[2]], dtype=np.uint8))
            mask2 = cv2.inRange(hsv, np.array([180 + lower_hue, lower_sv[1], lower_sv[2]], dtype=np.uint8), np.array([179, upper_sv[1], upper_sv[2]], dtype=np.uint8))
            return mask1 | mask2
        if upper_hue > 179:
            mask1 = cv2.inRange(hsv, np.array([lower_hue, lower_sv[1], lower_sv[2]], dtype=np.uint8), np.array([179, upper_sv[1], upper_sv[2]], dtype=np.uint8))
            mask2 = cv2.inRange(hsv, np.array([0, lower_sv[1], lower_sv[2]], dtype=np.uint8), np.array([upper_hue - 180, upper_sv[1], upper_sv[2]], dtype=np.uint8))
            return mask1 | mask2
        return cv2.inRange(
            hsv,
            np.array([lower_hue, lower_sv[1], lower_sv[2]], dtype=np.uint8),
            np.array([upper_hue, upper_sv[1], upper_sv[2]], dtype=np.uint8),
        )

    lower = np.array([0, 0, max(0, value - 60)], dtype=np.uint8)
    upper = np.array([179, min(80, saturation + 50), min(255, value + 60)], dtype=np.uint8)
    return cv2.inRange(hsv, lower, upper)


def contour_mean_hsv(hsv: np.ndarray, contour: np.ndarray) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    mean_hsv = cv2.mean(hsv, mask=mask)[:3]
    return np.array(mean_hsv, dtype=np.float32)


def deproject_pixel_to_point(intrinsics, pixel: tuple[int, int], depth_m: float) -> tuple[float, float, float]:
    if rs is not None:
        x, y, z = rs.rs2_deproject_pixel_to_point(intrinsics, [pixel[0], pixel[1]], depth_m)
        return float(x), float(y), float(z)

    fx = float(intrinsics.fx)
    fy = float(getattr(intrinsics, "fy", intrinsics.fx))
    ppx = float(getattr(intrinsics, "ppx", 0.0))
    ppy = float(getattr(intrinsics, "ppy", 0.0))
    x = (pixel[0] - ppx) * depth_m / fx
    y = (pixel[1] - ppy) * depth_m / fy
    return float(x), float(y), float(depth_m)


def find_square_candidates(
    color_image: np.ndarray,
    depth_frame,
    square_size_m: float,
    learned_hsv: np.ndarray | None,
) -> tuple[list[SquareDetection], np.ndarray]:
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)

    learned_mask = build_learned_color_mask(hsv, learned_hsv)
    if learned_mask is not None:
        mask = learned_mask
    else:
        # Saturated-object mask helps colored squares; edges handle white/black/low-saturation squares.
        saturation_mask = cv2.inRange(hsv, np.array([0, 35, 35], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))
        mask = cv2.bitwise_or(saturation_mask, edges)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    candidates: list[SquareDetection] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 80:
            continue

        rect = cv2.minAreaRect(contour)
        (center_x, center_y), (width_px, height_px), _ = rect
        if width_px <= 1 or height_px <= 1:
            continue

        aspect_ratio = max(width_px, height_px) / min(width_px, height_px)
        if aspect_ratio > 1.35:
            continue

        rect_area = width_px * height_px
        rectangularity = area / rect_area if rect_area > 0 else 0.0
        if rectangularity < 0.45:
            continue

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        if len(approx) < 4 or len(approx) > 8:
            continue

        cx = int(round(center_x))
        cy = int(round(center_y))
        depth_m = median_depth(depth_frame, cx, cy)
        if depth_m <= 0:
            continue

        side_px = float((width_px + height_px) / 2.0)
        expected_side_px = float(square_size_m * depth_intrin.fx / depth_m)
        if expected_side_px <= 1:
            continue

        size_ratio = side_px / expected_side_px
        if size_ratio < 0.35 or size_ratio > 2.8:
            continue

        x, y, z = deproject_pixel_to_point(depth_intrin, (cx, cy), depth_m)
        mean_hsv = contour_mean_hsv(hsv, contour)
        size_penalty = abs(math.log(max(size_ratio, 1e-6)))
        aspect_penalty = aspect_ratio - 1.0
        rectangularity_bonus = 1.0 - min(rectangularity, 1.0)
        score = size_penalty * 2.0 + aspect_penalty + rectangularity_bonus

        candidates.append(
            SquareDetection(
                contour=contour,
                center_px=(cx, cy),
                depth_m=depth_m,
                camera_xyz_m=(float(x), float(y), float(z)),
                rect_points=cv2.boxPoints(rect).astype(np.int32),
                side_px=side_px,
                expected_side_px=expected_side_px,
                score=score,
                mean_hsv=mean_hsv,
            )
        )

    candidates.sort(key=lambda candidate: candidate.score)
    return candidates, mask


def draw_detection(color_image: np.ndarray, detection: SquareDetection, square_size_m: float, learned_hsv: np.ndarray | None) -> None:
    x, y, z = detection.camera_xyz_m
    cx, cy = detection.center_px
    cv2.drawContours(color_image, [detection.rect_points], -1, (0, 255, 0), 2)
    cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

    lines = [
        f"X={x:.3f}m  Y={y:.3f}m  Z={z:.3f}m",
        f"square={square_size_m * 100:.1f}cm  side={detection.side_px:.0f}px/{detection.expected_side_px:.0f}px",
        f"learned HSV={np.round(learned_hsv).astype(int).tolist() if learned_hsv is not None else 'auto'}",
        "ESC: exit   R: reset color",
    ]
    for index, text in enumerate(lines):
        cv2.putText(color_image, text, (10, 30 + index * 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)


def detect_red_block(square_size_m: float | None = None):
    """Detect any-color square marker and print its camera-frame center."""
    if rs is None:
        raise RuntimeError("pyrealsense2 is required to run the camera detector.")

    if square_size_m is None:
        square_size_m = ask_square_size_m()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)
    learned_hsv: np.ndarray | None = None

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            candidates, mask = find_square_candidates(color_image, depth_frame, square_size_m, learned_hsv)

            if candidates:
                detection = candidates[0]
                learned_hsv = detection.mean_hsv
                draw_detection(color_image, detection, square_size_m, learned_hsv)
                x, y, z = detection.camera_xyz_m
                print(
                    f"Square center: X={x:.4f}m, Y={y:.4f}m, Z={z:.4f}m, "
                    f"side={square_size_m * 100:.1f}cm"
                )
            else:
                cv2.putText(color_image, "No square found", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(color_image, "Move square into view or press R to reset color", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            cv2.imshow("Calibration Square", color_image)
            cv2.imshow("Square Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key in (ord("r"), ord("R")):
                learned_hsv = None
                print("Color model reset.")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Detect a square calibration marker of any visible color.")
    parser.add_argument("--size-cm", type=float, default=None, help="Square side length in centimeters.")
    args = parser.parse_args()

    square_size_m = None if args.size_cm is None else args.size_cm / 100.0
    detect_red_block(square_size_m=square_size_m)


if __name__ == "__main__":
    main()
