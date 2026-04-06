#!/usr/bin/env python3
"""
地鼠检测可视化测试（纯视觉，无需机器人）
========================================

功能：
  1. 用 AprilTag 自动定位 9 个洞（若有 board_config.json）
     或从 mole_calibration.json 加载手动标定位置
  2. 实时显示地鼠检测结果

检测原理（绿色背景消失法）：
  - 地鼠弹出 → 遮住洞口上方的绿色背景 → 该区域绿色像素比例下降
  - 绿色比例 < 阈值 → 判定"有地鼠"
  - 每个洞的"监控区域"是以洞中心为基准的矩形区域

用法（从项目根目录运行）：
  python stage2_test/test_detection.py           # 自动选择 AprilTag 或手动标定
  python stage2_test/test_detection.py --manual  # 强制使用 mole_calibration.json

快捷键：
  [a] 自动调整监控区域大小（根据洞间距）
  [ESC] 退出
"""

import sys
import cv2
import time
import argparse
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from mole_detector import MoleDetector

# ──────────────────────────────────────────────────────────
#  AprilTag 定位（可选）
# ──────────────────────────────────────────────────────────

def try_apriltag_locate(camera_idx=0):
    """
    尝试用 AprilTag 获取洞位像素坐标。
    Returns (hole_centers, locator_or_None, depth_m_or_None)
    """
    try:
        from apriltag_locator import AprilTagLocator
    except ImportError:
        print("ℹ️  pupil-apriltags 未安装，跳过 AprilTag")
        return None, None, None

    config = "outputs/board_config.json"
    if not Path(config).exists():
        print("ℹ️  未找到 board_config.json，跳过 AprilTag")
        return None, None, None

    try:
        locator = AprilTagLocator.from_intrinsics_file(
            intrinsics_file="outputs/cheap_camera_intrinsics.json",
            config_file=config,
        )
    except FileNotFoundError as e:
        print(f"ℹ️  AprilTag 初始化失败: {e}")
        return None, None, None

    # 打开相机采一帧
    cap = cv2.VideoCapture(camera_idx)
    if not cap.isOpened():
        return None, None, None

    print("AprilTag 相机预热...", end="", flush=True)
    for _ in range(20):
        cap.read()
    print(" 完成")

    ret, frame = cap.read()
    cap.release()
    if not ret:
        return None, None, None

    try:
        hole_pixels, depth_m = locator.locate(frame)
        hole_centers = [(int(u), int(v)) for u, v in hole_pixels]
        print(f"✅ AprilTag 定位成功，深度={depth_m*1000:.0f}mm，找到 {len(hole_centers)} 个洞")
        return hole_centers, locator, depth_m
    except RuntimeError as e:
        print(f"⚠️  AprilTag 定位失败: {e}")
        return None, None, None


# ──────────────────────────────────────────────────────────
#  主函数
# ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="地鼠检测可视化测试")
    parser.add_argument("--manual", action="store_true", help="强制使用 mole_calibration.json")
    parser.add_argument("--cam", type=int, default=0, help="相机设备号（默认 0）")
    args = parser.parse_args()

    # ── 1. 加载检测器 ──────────────────────────────────────
    locator = None
    depth_m = None

    if not args.manual:
        hole_centers, locator, depth_m = try_apriltag_locate(args.cam)
        if hole_centers is not None:
            # 从 mole_calibration.json 继承 HSV 参数
            try:
                detector = MoleDetector.from_calibration("outputs/mole_calibration.json")
                detector.hole_centers = hole_centers
                detector.monitor_regions = detector._compute_monitor_regions()
                detector.auto_compute_monitor_size()
                print("✅ 使用 AprilTag 洞位 + mole_calibration.json 的 HSV 参数")
            except FileNotFoundError:
                detector = MoleDetector(hole_centers=hole_centers)
                detector.auto_compute_monitor_size()
                print("✅ 使用 AprilTag 洞位 + 默认 HSV 参数")
        else:
            locator = None

    if locator is None or args.manual:
        try:
            detector = MoleDetector.from_calibration("outputs/mole_calibration.json")
            print("✅ 使用 mole_calibration.json（手动标定）")
        except FileNotFoundError:
            print("❌ 未找到 mole_calibration.json")
            print("   请先运行: python stage2_test/mole_detector.py --calibrate")
            print("   或先运行: python stage2_test/apriltag_locator.py --setup")
            sys.exit(1)

    print(f"   洞中心: {detector.hole_centers}")
    print(f"   监控区域大小: {detector.monitor_width} x {detector.monitor_height} px")
    print(f"   绿色阈值: {detector.green_ratio_threshold:.0%}")

    # ── 2. 打开相机 ────────────────────────────────────────
    print(f"\n打开相机 {args.cam}...")
    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"❌ 无法打开相机 {args.cam}")
        sys.exit(1)

    print("相机预热...", end="", flush=True)
    for _ in range(20):
        cap.read()
    print(" 完成")

    print("\n[a] 自动调整监控区域  [ESC] 退出\n")

    # AprilTag 刷新计时
    _last_apriltag_t = time.time()
    _APRILTAG_INTERVAL = 3.0   # 每 3 秒刷新一次洞位
    _frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️  相机读取失败")
            time.sleep(0.05)
            continue

        _frame_count += 1

        # AprilTag 周期刷新
        if locator is not None and (time.time() - _last_apriltag_t > _APRILTAG_INTERVAL):
            try:
                hole_pixels, depth_m = locator.locate(frame)
                detector.hole_centers = [(int(u), int(v)) for u, v in hole_pixels]
                detector.monitor_regions = detector._compute_monitor_regions()
                _last_apriltag_t = time.time()
            except RuntimeError:
                pass   # 保持旧位置

        # 检测地鼠
        mole_indices, green_ratios = detector.detect(frame)

        # 绘制调试画面
        vis = detector.draw_debug(frame, mole_indices, green_ratios)

        # 在洞中心画小点（区分 AprilTag 点 vs 监控区域）
        for i, (cx, cy) in enumerate(detector.hole_centers):
            color = (0, 0, 255) if i in mole_indices else (0, 200, 0)
            cv2.circle(vis, (int(cx), int(cy)), 3, color, -1)

        # 顶部状态栏
        src = "AprilTag" if locator is not None else "manual"
        n_moles = len(mole_indices)
        status = f"[{src}]  moles={n_moles}  {[i+1 for i in mole_indices]}"
        cv2.putText(vis, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("Mole Detection Test", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break
        elif key == ord("a"):
            detector.auto_compute_monitor_size()
            print(f"监控区域已自动调整: {detector.monitor_width} x {detector.monitor_height} px")

    cap.release()
    cv2.destroyAllWindows()
    print("✅ 退出")


if __name__ == "__main__":
    main()
