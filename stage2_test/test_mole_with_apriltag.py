#!/usr/bin/env python3
"""
AprilTag + 地鼠检测 联合测试
==============================

与 mole_detector.py --test 的唯一区别：
  --test  : 洞口位置 = mole_calibration.json 里手动点击的静态坐标
  本脚本  : 洞口位置 = AprilTag 每帧实时计算的动态坐标（与 whack_a_mole.py 正式运行完全一致）
  HSV 参数、监控区域尺寸：两者都来自 mole_calibration.json

用途：在没有机器人/手眼标定的情况下，验证地鼠视觉识别效果是否满足要求。

使用方法（从项目根目录运行）：
  python stage2_test/test_mole_with_apriltag.py
"""

import sys
import json
import numpy as np
import cv2
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from apriltag_locator import AprilTagLocator
from mole_detector import MoleDetector

MOLE_CALIB_FILE   = "outputs/mole_calibration.json"
BOARD_CONFIG_FILE = "outputs/board_config.json"
INTRINSICS_FILE   = "outputs/cheap_camera_intrinsics.json"
APRILTAG_INTERVAL = 3   # 每 N 帧运行一次 AprilTag（降低 CPU 占用）


def main():
    print("=" * 60)
    print("  AprilTag + 地鼠检测 联合测试")
    print("=" * 60)

    # ── 1. 加载 MoleDetector（只用 HSV 参数 + 监控尺寸，洞口位置由 AprilTag 覆盖）
    try:
        detector = MoleDetector.from_calibration(MOLE_CALIB_FILE)
        print(f"\n✅ 检测参数已加载: {MOLE_CALIB_FILE}")
        print(f"   监控区域: {detector.monitor_width}×{detector.monitor_height}px  "
              f"Y偏移={detector.monitor_offset_y}px")
        print(f"   绿色 HSV: {detector.green_hsv_low.tolist()} ~ "
              f"{detector.green_hsv_high.tolist()}")
        print(f"   地鼠阈值: 绿色比例 < {detector.green_ratio_threshold:.0%}")
    except FileNotFoundError:
        print(f"\n❌ 未找到 {MOLE_CALIB_FILE}")
        print("   请先运行: python stage2_test/mole_detector.py --calibrate")
        sys.exit(1)

    # ── 2. 加载 AprilTagLocator
    try:
        locator = AprilTagLocator.from_intrinsics_file(
            intrinsics_file=INTRINSICS_FILE,
            config_file=BOARD_CONFIG_FILE,
        )
    except FileNotFoundError as e:
        print(f"\n❌ 文件不存在: {e}")
        sys.exit(1)
    except ImportError as e:
        print(f"\n❌ {e}")
        sys.exit(1)

    if locator.board_data is None:
        print(f"\n❌ 未找到底板配置: {BOARD_CONFIG_FILE}")
        print("   请先运行: python stage2_test/apriltag_locator.py --setup")
        sys.exit(1)

    # ── 3. 打开相机
    print("\n打开相机...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 无法打开相机 0，尝试相机 1...")
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            print("❌ 无法打开任何相机")
            sys.exit(1)

    # 预热（Linux 相机冷启动前几帧全黑）
    print("相机预热...", end="", flush=True)
    for _ in range(20):
        cap.read()
    print(" 完成")

    print("\n按 [q / ESC] 退出\n")

    cv2.namedWindow("Mole Det (AprilTag)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Mole Det (AprilTag)", 960, 540)

    frame_count  = 0
    apriltag_ok  = False
    depth_info   = ""

    while True:
        ret, frame = cap.read()
        if not ret:
            cv2.waitKey(50)
            continue

        frame_count += 1

        # ── 每 APRILTAG_INTERVAL 帧更新洞口位置 ──────────────
        if frame_count % APRILTAG_INTERVAL == 0:
            try:
                hole_pixels, depth_m = locator.locate(frame)
                # 更新检测器的洞口位置（与 whack_a_mole.py 运行时逻辑完全一致）
                detector.hole_centers = [(int(u), int(v)) for u, v in hole_pixels]
                detector.monitor_regions = detector._compute_monitor_regions()
                apriltag_ok = True
                depth_info  = f"AprilTag OK  depth={depth_m*1000:.0f}mm"
            except RuntimeError as e:
                apriltag_ok = False
                depth_info  = f"AprilTag: {e}"

        # ── 地鼠检测 + 可视化 ─────────────────────────────────
        if apriltag_ok:
            mole_indices, green_ratios = detector.detect(frame)
            debug = detector.draw_debug(frame, mole_indices, green_ratios)

            if mole_indices:
                holes_str = ", ".join(str(i + 1) for i in mole_indices)
                print(f"\r🔴 洞 {holes_str} 有地鼠   ", end="", flush=True)
            else:
                print(f"\r  （无地鼠）              ", end="", flush=True)
        else:
            debug = frame.copy()
            cv2.putText(
                debug, depth_info,
                (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
            )

        # 状态栏
        bar_color = (0, 220, 0) if apriltag_ok else (0, 0, 255)
        cv2.putText(
            debug, depth_info,
            (10, debug.shape[0] - 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_color, 1,
        )
        cv2.putText(
            debug, "[q] quit",
            (10, debug.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1,
        )

        cv2.imshow("Mole Det (AprilTag)", debug)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    print()
    cap.release()
    cv2.destroyAllWindows()
    print("✅ 测试结束")


if __name__ == "__main__":
    main()