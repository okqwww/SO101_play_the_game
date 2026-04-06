#!/usr/bin/env python3
"""
AprilTag 定位精度验证脚本
==========================

用途：
  验证 AprilTag 自动定位的精度：
    1. 通过 AprilTag 计算每个洞的像素坐标
    2. 转换为机械臂基座坐标系的 3D 坐标
    3. 对比「AprilTag 算出的位置」与「mole_calibration.json 中手动标定的位置」
    4. 检查底板平移后，AprilTag 是否能自动追踪（实时可视化）

使用方法（从项目根目录运行）：
  python stage2_test/error_analysis/check_apriltag_calib.py
"""

import sys
import json
import numpy as np
import cv2
from pathlib import Path

# 路径设置
ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT / "stage2_test"))

from apriltag_locator import AprilTagLocator
from coordinate_transformer import CoordinateTransformer


def load_manual_holes(calib_file="outputs/mole_calibration.json"):
    """加载手动标定的地鼠洞像素坐标"""
    p = ROOT / calib_file
    if not p.exists():
        return None
    with open(p) as f:
        d = json.load(f)
    return d.get("hole_centers", None)  # List of [x, y]


def main():
    print("=" * 60)
    print("AprilTag 定位精度验证")
    print("=" * 60)

    # 1. 初始化 AprilTagLocator
    try:
        locator = AprilTagLocator.from_intrinsics_file(
            intrinsics_file="outputs/cheap_camera_intrinsics.json",
            config_file="outputs/board_config.json",
        )
    except FileNotFoundError as e:
        print(f"❌ 文件不存在: {e}")
        print("   请先运行: python stage2_test/apriltag_locator.py --setup")
        sys.exit(1)
    except ImportError as e:
        print(f"❌ {e}")
        sys.exit(1)

    if locator.board_data is None:
        print("❌ 未找到底板配置，请先运行 setup_board()")
        sys.exit(1)

    # 2. 初始化坐标转换器
    try:
        transformer = CoordinateTransformer()
    except FileNotFoundError as e:
        print(f"❌ 坐标转换器初始化失败: {e}")
        sys.exit(1)

    # 3. 加载手动标定数据（可选对比）
    manual_holes = load_manual_holes()
    if manual_holes:
        print(f"\n✅ 找到手动标定数据：{len(manual_holes)} 个洞")
    else:
        print("\nℹ️  未找到手动标定数据（outputs/mole_calibration.json），仅显示 AprilTag 结果")

    # 4. 打开相机
    print("\n打开相机中...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 无法打开相机 0，尝试相机 1...")
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            print("❌ 无法打开任何相机")
            sys.exit(1)

    # 相机预热：丢弃前 20 帧（Linux 相机冷启动前几帧全黑）
    print("相机预热中...", end="", flush=True)
    for _ in range(20):
        cap.read()
    print(" 完成")

    print("按 [空格] 拍摄当前帧进行分析")
    print("按 [r]    实时追踪模式（观察底板移动后的自动重定位）")
    print("按 [s]    切回快照模式")
    print("按 [ESC]  退出\n")

    mode = "snapshot"  # "snapshot" or "realtime"
    _detect_interval = 3   # 实时模式每 N 帧运行一次 AprilTag 检测（降低 CPU 占用）
    _frame_count = 0
    _last_realtime_vis = None  # 缓存上一次成功的检测结果画面

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️  相机读取失败，重试...")
            cv2.waitKey(100)
            continue

        _frame_count += 1
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord("r"):
            mode = "realtime"
            print("切换到实时追踪模式（每{}帧检测一次）".format(_detect_interval))
        elif key == ord("s"):
            mode = "snapshot"
            _last_realtime_vis = None
            print("切换到快照模式")

        if mode == "realtime":
            # 每 N 帧运行一次检测，其他帧复用上次结果
            if _frame_count % _detect_interval == 0:
                try:
                    hole_pixels, depth_m = locator.locate(frame)
                    vis = locator.draw_detection(frame, hole_pixels, depth_m)

                    # 与手动标定对比（画出手动点）
                    if manual_holes:
                        for i, (mx, my) in enumerate(manual_holes):
                            cv2.circle(vis, (int(mx), int(my)), 3, (0, 0, 255), -1)
                            cv2.putText(
                                vis, f"M{i+1}",
                                (int(mx) - 20, int(my) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
                            )

                    cv2.putText(
                        vis, "[green]AprilTag  [red]Manual  [r/s]mode  [ESC]quit",
                        (10, vis.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    )
                    _last_realtime_vis = vis
                except RuntimeError as e:
                    vis = frame.copy()
                    cv2.putText(
                        vis, f"No tag: {e}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                    )
                    _last_realtime_vis = vis

            # 显示（使用缓存或当前帧作为背景）
            if _last_realtime_vis is not None:
                cv2.imshow("AprilTag Check", _last_realtime_vis)
            else:
                cv2.imshow("AprilTag Check", frame)

        elif key == ord(" "):
            # 快照分析
            print("\n" + "─" * 60)
            print("快照分析")
            print("─" * 60)

            try:
                hole_pixels, depth_m = locator.locate(frame)
                print(f"✅ AprilTag 定位成功")
                print(f"   深度: {depth_m * 1000:.1f} mm  "
                      f"(原 depth_camera: {transformer.depth_camera * 1000:.1f} mm, "
                      f"差值: {(depth_m - transformer.depth_camera) * 1000:+.1f} mm)")

            except RuntimeError as e:
                print(f"❌ 定位失败: {e}")
                continue

            # 更新 transformer 深度（临时）
            old_depth = transformer.depth_camera
            transformer.depth_camera = depth_m
            transformer.table_height = (
                transformer.T_cam_to_base[:3, 3][2] - depth_m
            )

            print(f"\n{'洞':>3} {'AprilTag像素':>18} {'AprilTag基座XY(mm)':>22}", end="")
            if manual_holes:
                print(f" {'手动像素':>16} {'像素差ΔXY':>14}", end="")
            print()
            print("─" * (72 + (32 if manual_holes else 0)))

            for i, (au, av) in enumerate(hole_pixels):
                # AprilTag → 基座坐标
                pos = transformer.pixel_to_base_3d(au, av)

                line = (f"{i+1:>3} "
                        f"({au:6.1f}, {av:6.1f})  "
                        f"({pos[0]*1000:+7.1f}, {pos[1]*1000:+7.1f})")

                if manual_holes and i < len(manual_holes):
                    mx, my = manual_holes[i]
                    du = au - mx
                    dv = av - my
                    line += f"   ({mx:6.1f}, {my:6.1f})   ({du:+5.1f}, {dv:+5.1f})"

                print(line)

            # 恢复深度
            transformer.depth_camera = old_depth
            transformer.table_height = (
                transformer.T_cam_to_base[:3, 3][2] - old_depth
            )

            if manual_holes:
                # 计算像素误差统计
                errors = []
                for i, (au, av) in enumerate(hole_pixels[:len(manual_holes)]):
                    mx, my = manual_holes[i]
                    errors.append(np.sqrt((au-mx)**2 + (av-my)**2))
                print(f"\n像素误差统计: 均值={np.mean(errors):.1f}px  "
                      f"最大={np.max(errors):.1f}px  最小={np.min(errors):.1f}px")

            # 显示快照结果
            vis = locator.draw_detection(frame, hole_pixels, depth_m)
            if manual_holes:
                for mx, my in manual_holes:
                    cv2.circle(vis, (int(mx), int(my)), 3, (0, 0, 255), -1)
            cv2.imshow("AprilTag Check", vis)

        else:
            # 默认快照模式：直接显示原始帧（不跑检测器，保持流畅）
            vis = frame.copy()
            cv2.putText(
                vis, "[SPACE] analyze  [r] realtime  [ESC] quit",
                (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2,
            )
            cv2.imshow("AprilTag Check", vis)

    cap.release()
    cv2.destroyAllWindows()
    print("\n✅ 验证完成")


if __name__ == "__main__":
    main()
