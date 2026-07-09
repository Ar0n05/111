# coding=utf-8

"""
Eye-in-hand calibration by fitting a known object position.

This is the automated version of the original jing method.  It estimates the
camera-to-end transform from samples shaped as:

    camera_x, camera_y, camera_z, end_x, end_y, end_z, end_rx, end_ry, end_rz

By default it reads the latest file under:

    jing_data/eye_hand_data/data*/poses.txt

The public func() API returns (rotation_matrix, translation_vector), matching
the official compute_in_hand.py style.
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

try:
    from libs.log_setting import CommonLog
except ImportError:
    from .libs.log_setting import CommonLog


np.set_printoptions(precision=8, suppress=True)

SCRIPT_DIR = Path(__file__).resolve().parent
JING_DATA_ROOT = SCRIPT_DIR / "jing_data" / "eye_hand_data"
DEFAULT_RESULT_PATH = SCRIPT_DIR / "hand_eye_result_jing.yaml"
DEFAULT_EXPECTED_OBJ_BASE = np.array([0.4, 0.2, 0.03], dtype=float)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


@dataclass(frozen=True)
class JingCalibrationResult:
    rotation_matrix: np.ndarray
    translation_vector: np.ndarray
    total_error: float
    avg_error: float
    per_sample_errors: np.ndarray
    expected_obj_base: np.ndarray
    sample_count: int
    source_file: Path | None

    @property
    def avg_error_mm(self) -> float:
        return self.avg_error * 1000.0


def euler_to_rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    return R.from_euler("xyz", [rx, ry, rz], degrees=False).as_matrix()


def pose_to_homogeneous_matrix(pose: Iterable[float]) -> np.ndarray:
    x, y, z, rx, ry, rz = pose
    transform = np.eye(4)
    transform[:3, :3] = euler_to_rotation_matrix(rx, ry, rz)
    transform[:3, 3] = [x, y, z]
    return transform


def camera_to_end_transform(params: Iterable[float]) -> np.ndarray:
    rx_cam, ry_cam, rz_cam, tx, ty, tz = params
    transform = np.eye(4)
    transform[:3, :3] = euler_to_rotation_matrix(rx_cam, ry_cam, rz_cam)
    transform[:3, 3] = [tx, ty, tz]
    return transform


def _numeric_values(line: str) -> list[float]:
    return [float(value) for value in re.split(r"[\s,;]+", line.strip()) if value]


def _data_folder_sort_key(path: Path) -> tuple[str, int]:
    match = re.match(r"^data(\d{8})(\d*)$", path.name)
    if match is None:
        return ("", -1)
    suffix = int(match.group(2) or 0)
    return (match.group(1), suffix)


def find_latest_jing_data_folder(root: Path = JING_DATA_ROOT) -> Path | None:
    if not root.exists():
        return None

    folders = [
        folder
        for folder in root.iterdir()
        if folder.is_dir() and re.match(r"^data(\d{8})(\d*)$", folder.name)
    ]
    if not folders:
        return None

    return sorted(folders, key=_data_folder_sort_key, reverse=True)[0]


def _candidate_data_files(data_folder: Path) -> list[Path]:
    preferred_names = [
        "jing_samples.csv",
        "jing_samples.txt",
        "red_block_samples.csv",
        "red_block_samples.txt",
        "calibration_data.csv",
        "calibration_data.txt",
        "poses.csv",
        "poses.txt",
    ]
    candidates = [data_folder / name for name in preferred_names]
    return [path for path in candidates if path.exists()]


def resolve_data_file(data_file: str | os.PathLike[str] | None = None) -> Path | None:
    if data_file is not None:
        path = Path(data_file)
        if not path.is_absolute():
            path = SCRIPT_DIR / path
        if not path.exists():
            raise FileNotFoundError(f"Data file does not exist: {path}")
        return path

    latest_folder = find_latest_jing_data_folder()
    if latest_folder is None:
        return None

    candidates = _candidate_data_files(latest_folder)
    return candidates[0] if candidates else None


def _read_plain_samples(path: Path) -> list[list[float]]:
    samples: list[list[float]] = []
    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue

            try:
                values = _numeric_values(stripped)
            except ValueError:
                # Allows a simple header row.
                continue

            if len(values) == 9:
                samples.append(values)
            elif values:
                raise ValueError(f"{path}:{line_number} must contain 9 values, got {len(values)}")

    return samples


def _read_csv_samples(path: Path) -> list[list[float]]:
    with path.open("r", encoding="utf-8", newline="") as file:
        preview = file.read(1024)
        file.seek(0)
        has_header = csv.Sniffer().has_header(preview) if preview.strip() else False

        if has_header:
            reader = csv.DictReader(file)
            column_groups = [
                ("camera_x", "camera_y", "camera_z", "end_x", "end_y", "end_z", "end_rx", "end_ry", "end_rz"),
                ("x", "y", "z", "x1", "y1", "z1", "rx", "ry", "rz"),
            ]
            samples: list[list[float]] = []
            for row in reader:
                for columns in column_groups:
                    if all(column in row for column in columns):
                        samples.append([float(row[column]) for column in columns])
                        break
                else:
                    raise ValueError(f"{path} header must contain camera/end pose columns")
            return samples

    return _read_plain_samples(path)


def load_calibration_samples(data_file: str | os.PathLike[str] | None = None) -> tuple[list[list[float]], Path | None]:
    resolved_file = resolve_data_file(data_file)
    if resolved_file is None:
        script_dir = str(SCRIPT_DIR)
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)
        from compute_in_hand_12data import calibration_data

        logger_.warning("No jing data file found; using compute_in_hand_12data.calibration_data fallback")
        return [list(row) for row in calibration_data], None

    if resolved_file.suffix.lower() in {".yaml", ".yml"}:
        with resolved_file.open("r", encoding="utf-8") as file:
            content = yaml.safe_load(file) or {}
        samples = content.get("calibration_data") or content.get("samples")
        if not samples:
            raise ValueError(f"{resolved_file} does not contain calibration_data or samples")
        return [list(map(float, row)) for row in samples], resolved_file

    if resolved_file.suffix.lower() == ".csv":
        samples = _read_csv_samples(resolved_file)
    else:
        samples = _read_plain_samples(resolved_file)

    if not samples:
        raise ValueError(f"No calibration samples found in {resolved_file}")

    return samples, resolved_file


def preprocess_data(data: Iterable[Iterable[float]], expected_obj_base: np.ndarray | None = None):
    obj_camera_list = []
    t_base_to_end_list = []

    for index, item in enumerate(data, start=1):
        values = list(map(float, item))
        if len(values) != 9:
            raise ValueError(f"Sample {index} must contain 9 values, got {len(values)}")

        x, y, z, x1, y1, z1, rx, ry, rz = values
        obj_camera_list.append(np.array([x, y, z, 1.0], dtype=float))
        t_base_to_end_list.append(pose_to_homogeneous_matrix([x1, y1, z1, rx, ry, rz]))

    return obj_camera_list, t_base_to_end_list


def error_function(params, obj_camera_list, t_base_to_end_list, actual_obj_base):
    t_camera_to_end = camera_to_end_transform(params)

    errors = []
    for obj_camera, t_base_to_end in zip(obj_camera_list, t_base_to_end_list):
        obj_end = t_camera_to_end @ obj_camera
        obj_base = t_base_to_end @ obj_end
        errors.extend(obj_base[:3] - actual_obj_base)

    return np.array(errors, dtype=float)


def optimize_transform(data, actual_obj_base, initial_params: np.ndarray | None = None):
    samples = list(data)
    if len(samples) < 3:
        raise ValueError("At least 3 samples are required; 10 or more diverse poses are recommended.")

    obj_camera_list, t_base_to_end_list = preprocess_data(samples, actual_obj_base)
    if initial_params is None:
        initial_params = np.zeros(6, dtype=float)

    method = "lm" if len(samples) * 3 >= 6 else "trf"
    result = least_squares(
        error_function,
        initial_params,
        args=(obj_camera_list, t_base_to_end_list, actual_obj_base),
        method=method,
    )

    optimized_rotation = euler_to_rotation_matrix(*result.x[:3])
    optimized_translation = result.x[3:]
    residuals = error_function(result.x, obj_camera_list, t_base_to_end_list, actual_obj_base)
    total_error = float(np.linalg.norm(residuals))
    avg_error = total_error / len(samples)

    return optimized_rotation, optimized_translation, total_error, avg_error


def compute_sample_errors(rotation_matrix, translation_vector, data, expected_obj_base) -> np.ndarray:
    t_camera_to_end = np.eye(4)
    t_camera_to_end[:3, :3] = rotation_matrix
    t_camera_to_end[:3, 3] = np.asarray(translation_vector).reshape(3)

    errors = []
    for item in data:
        x, y, z, x1, y1, z1, rx, ry, rz = list(map(float, item))
        obj_camera = np.array([x, y, z, 1.0], dtype=float)
        t_base_to_end = pose_to_homogeneous_matrix([x1, y1, z1, rx, ry, rz])
        obj_base = t_base_to_end @ (t_camera_to_end @ obj_camera)
        errors.append(float(np.linalg.norm(obj_base[:3] - expected_obj_base)))

    return np.array(errors, dtype=float)


def verify_results(rotation_matrix, translation_vector, data, expected_obj_base):
    print("\nPer-sample verification")
    print("=" * 88)
    print(f"{'idx':<6} {'computed base xyz':<36} {'expected base xyz':<28} {'error(mm)':>10}")
    print("=" * 88)

    t_camera_to_end = np.eye(4)
    t_camera_to_end[:3, :3] = rotation_matrix
    t_camera_to_end[:3, 3] = np.asarray(translation_vector).reshape(3)

    errors = []
    for index, item in enumerate(data, start=1):
        x, y, z, x1, y1, z1, rx, ry, rz = list(map(float, item))
        obj_camera = np.array([x, y, z, 1.0], dtype=float)
        t_base_to_end = pose_to_homogeneous_matrix([x1, y1, z1, rx, ry, rz])
        obj_base = t_base_to_end @ (t_camera_to_end @ obj_camera)
        error = float(np.linalg.norm(obj_base[:3] - expected_obj_base))
        errors.append(error)
        print(
            f"{index:<6} "
            f"[{obj_base[0]: .4f}, {obj_base[1]: .4f}, {obj_base[2]: .4f}]   "
            f"[{expected_obj_base[0]: .4f}, {expected_obj_base[1]: .4f}, {expected_obj_base[2]: .4f}]   "
            f"{error * 1000:10.2f}"
        )

    print("=" * 88)
    avg_error = float(np.mean(errors))
    print(f"Average error: {avg_error:.6f} m ({avg_error * 1000:.2f} mm)")
    return np.array(errors, dtype=float)


def save_result(result: JingCalibrationResult, result_file: str | os.PathLike[str] | None = DEFAULT_RESULT_PATH) -> Path:
    path = Path(result_file) if result_file is not None else DEFAULT_RESULT_PATH
    if not path.is_absolute():
        path = SCRIPT_DIR / path

    quaternion = R.from_matrix(result.rotation_matrix).as_quat()
    payload = {
        "rotation_matrix": result.rotation_matrix.tolist(),
        "translation_vector": result.translation_vector.reshape(3).tolist(),
        "quaternion": quaternion.tolist(),
        "expected_obj_base": result.expected_obj_base.tolist(),
        "sample_count": result.sample_count,
        "source_file": str(result.source_file) if result.source_file else None,
        "total_error_m": float(result.total_error),
        "avg_error_m": float(result.avg_error),
        "avg_error_mm": float(result.avg_error_mm),
        "per_sample_error_mm": (result.per_sample_errors * 1000.0).tolist(),
    }

    with path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(payload, file, allow_unicode=True, sort_keys=False)

    logger_.info(f"Saved result to {path}")
    return path


def parse_expected_obj_base(value: str | Iterable[float] | None) -> np.ndarray:
    if value is None:
        return DEFAULT_EXPECTED_OBJ_BASE.copy()

    if isinstance(value, str):
        values = _numeric_values(value)
    else:
        values = list(map(float, value))

    if len(values) != 3:
        raise ValueError("expected_obj_base must contain exactly 3 values")

    return np.array(values, dtype=float)


def run_calibration(
    data_file: str | os.PathLike[str] | None = None,
    expected_obj_base: str | Iterable[float] | None = None,
    result_file: str | os.PathLike[str] | None = DEFAULT_RESULT_PATH,
    verify: bool = True,
) -> JingCalibrationResult:
    expected = parse_expected_obj_base(expected_obj_base)
    data, source_file = load_calibration_samples(data_file)

    logger_.info(f"Using {len(data)} jing samples")
    if source_file is not None:
        logger_.info(f"Source data file: {source_file}")
    logger_.info(f"Expected object base coordinate: {expected}")

    rotation_matrix, translation_vector, total_error, avg_error = optimize_transform(data, expected)
    per_sample_errors = compute_sample_errors(rotation_matrix, translation_vector, data, expected)

    result = JingCalibrationResult(
        rotation_matrix=rotation_matrix,
        translation_vector=translation_vector.reshape(3, 1),
        total_error=total_error,
        avg_error=avg_error,
        per_sample_errors=per_sample_errors,
        expected_obj_base=expected,
        sample_count=len(data),
        source_file=source_file,
    )

    if verify:
        verify_results(rotation_matrix, translation_vector, data, expected)

    save_result(result, result_file)
    return result


def func(
    data_file: str | os.PathLike[str] | None = None,
    expected_obj_base: str | Iterable[float] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    result = run_calibration(data_file=data_file, expected_obj_base=expected_obj_base, verify=False)
    return result.rotation_matrix, result.translation_vector


def get_user_input():
    print("=" * 80)
    print("Jing hand-eye calibration manual input")
    print("Each sample: camera x y z + end x y z rx ry rz")
    print("=" * 80)

    while True:
        try:
            num_data = int(input("Number of samples (at least 3, recommended >= 10): "))
            if num_data >= 3:
                break
            print("Need at least 3 samples.")
        except ValueError:
            print("Please enter a valid integer.")

    expected_obj_base = parse_expected_obj_base(input("Expected object base xyz, comma separated: "))
    data = []
    for index in range(num_data):
        while True:
            try:
                line = input(f"Sample {index + 1}, 9 comma/space separated values: ")
                sample = _numeric_values(line)
                if len(sample) != 9:
                    raise ValueError
                data.append(sample)
                break
            except ValueError:
                print("Please enter exactly 9 numeric values.")

    return data, expected_obj_base


def main():
    parser = argparse.ArgumentParser(description="Jing eye-in-hand calibration from red-block samples.")
    parser.add_argument("--data-file", default=None, help="Optional sample file. Defaults to latest jing_data data*/poses.txt.")
    parser.add_argument(
        "--expected-obj-base",
        default=None,
        help="Known object coordinate in robot base frame, e.g. 0.4,0.2,0.03.",
    )
    parser.add_argument("--result-file", default=str(DEFAULT_RESULT_PATH), help="YAML output path.")
    parser.add_argument("--interactive", action="store_true", help="Use manual input instead of a data file.")
    parser.add_argument("--no-verify", action="store_true", help="Do not print per-sample verification table.")
    args = parser.parse_args()

    if args.interactive:
        data, expected = get_user_input()
        rotation_matrix, translation_vector, total_error, avg_error = optimize_transform(data, expected)
        errors = verify_results(rotation_matrix, translation_vector, data, expected)
        result = JingCalibrationResult(
            rotation_matrix=rotation_matrix,
            translation_vector=translation_vector.reshape(3, 1),
            total_error=total_error,
            avg_error=avg_error,
            per_sample_errors=errors,
            expected_obj_base=expected,
            sample_count=len(data),
            source_file=None,
        )
        save_result(result, args.result_file)
    else:
        result = run_calibration(
            data_file=args.data_file,
            expected_obj_base=args.expected_obj_base,
            result_file=args.result_file,
            verify=not args.no_verify,
        )

    print("\nFinal camera-to-end transform")
    print("rotation_matrix = np.array(")
    print(result.rotation_matrix)
    print(")")
    print(f"translation_vector = np.array({result.translation_vector.reshape(3).tolist()})")
    print(f"avg_error = {result.avg_error:.6f} m ({result.avg_error_mm:.2f} mm)")


if __name__ == "__main__":
    main()
