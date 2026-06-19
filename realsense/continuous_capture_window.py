#!/usr/bin/env python3
from pathlib import Path
import glob
import time

import cv2
import numpy as np
import pyrealsense2 as rs

# pip install pyrealsense2 opencv-python numpy

# cd $env:USERPROFILE\Desktop\realsense
# python .\continuous_capture_window.py

# SPACE: start/stop continuous image capture
# ENTER: save one image
# ESC: quit

WIDTH = 640
HEIGHT = 480
FPS = 15
FRAME_TIMEOUT_MS = 15000
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
JPEG_QUALITY = 95
START_DELAY_SECONDS = 2.0

AUTO_EXPOSURE = False
EXPOSURE = 155
AUTO_WHITE_BALANCE = False
WHITE_BALANCE = 4000.0

SATURATION = 55
CONTRAST = 55
GAMMA = 300
SHARPNESS = 30
GAIN = 64

SAVE_DIR = Path(__file__).resolve().parent / "dataset" / "images" / "train"

STREAM_CANDIDATES = [
    (WIDTH, HEIGHT, rs.format.bgr8, FPS),
    (WIDTH, HEIGHT, rs.format.rgb8, FPS),
    (640, 480, rs.format.bgr8, 30),
    (640, 480, rs.format.rgb8, 30),
    (424, 240, rs.format.bgr8, 15),
    (320, 240, rs.format.bgr8, 15),
]


def set_option_if_supported(sensor, option, value):
    if sensor is not None and sensor.supports(option):
        sensor.set_option(option, value)


def find_color_sensor(profile):
    device = profile.get_device()
    for sensor in device.query_sensors():
        for stream_profile in sensor.get_stream_profiles():
            if stream_profile.stream_type() == rs.stream.color:
                return sensor
    return None


def next_image_index(save_dir):
    image_indices = []
    for image_path in glob.glob(str(save_dir / "img_*.jpg")):
        try:
            image_indices.append(int(Path(image_path).stem.split("_")[1]))
        except (IndexError, ValueError):
            continue

    if not image_indices:
        return 0
    return max(image_indices) + 1


def image_file_count(save_dir):
    count = 0
    for image_path in glob.glob(str(save_dir / "img_*.jpg")):
        try:
            int(Path(image_path).stem.split("_")[1])
        except (IndexError, ValueError):
            continue
        count += 1
    return count


def save_jpeg(path, image):
    success, encoded_image = cv2.imencode(
        ".jpg",
        image,
        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
    )
    if not success:
        return False

    encoded_image.tofile(str(path))
    return path.exists() and path.stat().st_size > 0


def print_connected_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        raise RuntimeError("RealSense camera was not detected.")

    for device in devices:
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        usb_type = (
            device.get_info(rs.camera_info.usb_type_descriptor)
            if device.supports(rs.camera_info.usb_type_descriptor)
            else "unknown"
        )
        print(f"Camera: {name} / serial: {serial} / USB: {usb_type}")


def start_color_pipeline():
    last_error = None

    for width, height, color_format, fps in STREAM_CANDIDATES:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, color_format, fps)

        try:
            profile = pipeline.start(config)
            print(f"Started color stream: {width}x{height} {color_format} {fps}fps")
            return pipeline, profile, color_format
        except RuntimeError as exc:
            last_error = exc
            try:
                pipeline.stop()
            except RuntimeError:
                pass

    raise RuntimeError(f"Could not start a color stream. Last error: {last_error}")


def apply_camera_settings(profile):
    color_sensor = find_color_sensor(profile)

    set_option_if_supported(color_sensor, rs.option.enable_auto_exposure, 1.0 if AUTO_EXPOSURE else 0.0)
    if not AUTO_EXPOSURE:
        set_option_if_supported(color_sensor, rs.option.exposure, float(EXPOSURE))

    set_option_if_supported(color_sensor, rs.option.enable_auto_white_balance, 1.0 if AUTO_WHITE_BALANCE else 0.0)
    if not AUTO_WHITE_BALANCE:
        set_option_if_supported(color_sensor, rs.option.white_balance, float(WHITE_BALANCE))

    set_option_if_supported(color_sensor, rs.option.saturation, float(SATURATION))
    set_option_if_supported(color_sensor, rs.option.contrast, float(CONTRAST))
    set_option_if_supported(color_sensor, rs.option.gamma, float(GAMMA))
    set_option_if_supported(color_sensor, rs.option.sharpness, float(SHARPNESS))
    set_option_if_supported(color_sensor, rs.option.gain, float(GAIN))


def draw_status(image, is_recording, countdown_left, session_count, total_count, image_count, capture_fps):
    if countdown_left is not None:
        status = f"START IN {countdown_left:.1f}s"
        color = (0, 220, 255)
    elif is_recording:
        status = "REC"
        color = (0, 0, 255)
    else:
        status = "READY"
        color = (0, 180, 0)

    cv2.rectangle(image, (8, 8), (330, 96), (0, 0, 0), -1)
    cv2.putText(image, status, (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    cv2.putText(
        image,
        f"total {total_count} / session {session_count}",
        (20, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        image,
        f"next img_{image_count:04}.jpg",
        (20, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        image,
        f"{capture_fps:.1f} fps",
        (DISPLAY_WIDTH - 86, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )


def main():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    image_count = next_image_index(SAVE_DIR)
    total_count = image_file_count(SAVE_DIR)
    session_count = 0
    is_recording = False
    countdown_until = None
    last_frame_time = time.perf_counter()
    capture_fps = 0.0

    print_connected_devices()
    pipeline, profile, color_format = start_color_pipeline()
    apply_camera_settings(profile)

    print(f"Save directory: {SAVE_DIR}")
    print(f"Existing images: {total_count}")
    print(f"Next image: img_{image_count:04}.jpg")
    print("Press SPACE to start after 2s countdown / stop capture, ENTER to save one image, ESC to quit.")
    cv2.namedWindow("Continuous Capture", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError:
                print(
                    f"No frame arrived within {FRAME_TIMEOUT_MS / 1000:.0f}s. "
                    "Check that the RealSense camera is connected, not open in "
                    "another app, and on a USB 3 port."
                )
                key = cv2.waitKey(10)
                if key == 27:
                    break
                continue

            now = time.perf_counter()
            elapsed = now - last_frame_time
            last_frame_time = now
            if elapsed > 0:
                capture_fps = 0.9 * capture_fps + 0.1 * (1.0 / elapsed)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            if countdown_until is not None and now >= countdown_until:
                countdown_until = None
                is_recording = True
                print("Continuous capture started.")

            color_image = np.asanyarray(color_frame.get_data())
            if color_format == rs.format.rgb8:
                bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            else:
                bgr = color_image

            capture_image = cv2.resize(bgr, (DISPLAY_WIDTH, DISPLAY_HEIGHT))

            if is_recording:
                filename = SAVE_DIR / f"img_{image_count:04}.jpg"
                if save_jpeg(filename, capture_image):
                    session_count += 1
                    total_count += 1
                    image_count += 1
                    print(f"Saved: {filename}")
                else:
                    print(f"Failed to save: {filename}")

            display_image = capture_image.copy()
            countdown_left = None
            if countdown_until is not None:
                countdown_left = max(0.0, countdown_until - now)

            draw_status(
                display_image,
                is_recording,
                countdown_left,
                session_count,
                total_count,
                image_count,
                capture_fps,
            )
            cv2.imshow("Continuous Capture", display_image)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key in (10, 13):
                filename = SAVE_DIR / f"img_{image_count:04}.jpg"
                if save_jpeg(filename, capture_image):
                    session_count += 1
                    total_count += 1
                    image_count += 1
                    print(f"Saved single image: {filename}")
                else:
                    print(f"Failed to save: {filename}")
            if key == 32:
                if is_recording:
                    is_recording = False
                    countdown_until = None
                    print(f"Continuous capture stopped. Saved this session: {session_count}")
                elif countdown_until is not None:
                    countdown_until = None
                    print("Start countdown canceled.")
                else:
                    countdown_until = time.perf_counter() + START_DELAY_SECONDS
                    print(f"Continuous capture will start in {START_DELAY_SECONDS:.0f}s.")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
