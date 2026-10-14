"""Convert other/badapple.mp4 to 128x64 monochrome OLED frames."""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

WIDTH = 128
HEIGHT = 64
FPS = 12

SCRIPT_DIR = Path(__file__).resolve().parent
VIDEO_DIR = SCRIPT_DIR.parents[1] / "other"
SOURCE_VIDEO = VIDEO_DIR / "badapple.mp4"
OUTPUT_BIN = VIDEO_DIR / "BadApple_full_128x64_12fps.bin"
OUTPUT_INFO = VIDEO_DIR / "BadApple_full_128x64_12fps.json"


def pack_ssd1306(binary_image: np.ndarray) -> bytes:
    """Pack a 64x128 binary image into SSD1306 page order."""
    pixels = binary_image.reshape(8, 8, WIDTH)
    packed = np.zeros((8, WIDTH), dtype=np.uint8)
    for bit in range(8):
        packed |= ((pixels[:, bit, :] != 0).astype(np.uint8) << bit)
    return packed.tobytes()


def convert_source_cycle() -> tuple[bytearray, float]:
    """Convert one complete source-video cycle at the selected frame rate."""
    capture = cv2.VideoCapture(str(SOURCE_VIDEO))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {SOURCE_VIDEO}")

    source_fps = capture.get(cv2.CAP_PROP_FPS)
    source_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = source_frames / source_fps
    target_frames = int(duration * FPS)
    cycle = bytearray()

    for target_index in range(target_frames):
        capture.set(cv2.CAP_PROP_POS_MSEC, target_index * 1000.0 / FPS)
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(
                f"Decode failed at {target_index / FPS:.2f} seconds"
            )
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
        _, binary = cv2.threshold(
            resized, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
        )
        cycle.extend(pack_ssd1306(binary))
        if (target_index + 1) % (FPS * 10) == 0:
            print(
                f"Convert source: {(target_index + 1) / FPS:.0f}/{duration:.0f}s",
                flush=True,
            )

    capture.release()
    return cycle, duration


def write_output(cycle: bytearray, source_duration: float) -> None:
    bytes_per_frame = WIDTH * HEIGHT // 8
    cycle_frames = len(cycle) // bytes_per_frame

    with OUTPUT_BIN.open("wb") as output:
        output.write(cycle)

    metadata = {
        "width": WIDTH,
        "height": HEIGHT,
        "fps": FPS,
        "duration_seconds": cycle_frames / FPS,
        "frame_count": cycle_frames,
        "bytes_per_frame": bytes_per_frame,
        "source_duration_seconds": source_duration,
        "source_video": str(SOURCE_VIDEO),
        "host_playback_loop": True,
        "byte_order": "SSD1306 page order, vertical LSB first",
    }
    OUTPUT_INFO.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print(f"Output: {OUTPUT_BIN}", flush=True)
    print(f"Size: {OUTPUT_BIN.stat().st_size} bytes", flush=True)


def main() -> None:
    if not SOURCE_VIDEO.is_file():
        raise FileNotFoundError(f"Source video not found: {SOURCE_VIDEO}")
    cycle, source_duration = convert_source_cycle()
    write_output(cycle, source_duration)


if __name__ == "__main__":
    main()
