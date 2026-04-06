#!/usr/bin/env python3
"""
单次点击地鼠
============

流程: 捕获一帧 → 检测地鼠 → 点击第一个地鼠 → 回到初始位置 → 退出

使用方法（从项目根目录运行）:
  python stage2_test/tap_one_mole.py

前提:
  - 已完成洞口标定: python stage2_test/mole_detector.py --calibrate
  - outputs/mole_calibration.json 存在
  - outputs/cheap_camera_intrinsics.json 等标定文件存在
  - 可选（推荐）: outputs/board_config.json (AprilTag底板配置)
"""

import sys
import json
import time
import numpy as np

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.cameras.configs import ColorMode

# stage2_test 下的模块（运行时 Python 自动将脚本目录加入 sys.path）
from coordinate_transformer import CoordinateTransformer
from mole_detector import MoleDetector
from whack_a_mole import solve_ik, move_robot, capture_frame_bgr
from fk_corrector import FKCorrector

# AprilTag 自动定位（可选）
try:
    from apriltag_locator import AprilTagLocator
    _APRILTAG_AVAILABLE = True
except ImportError:
    _APRILTAG_AVAILABLE = False

# ═══════════════════════════════════════════════════════════
#  配置参数（与 whack_a_mole.py 保持一致）
# ═══════════════════════════════════════════════════════════

SAFE_HEIGHT_OFFSET = 0.040   # 安全高度：桌面上方 40mm
TAP_DEPTH_OFFSET   = -0.001   # 点击深度：桌面以下 2mm
MOVE_WAIT_TIME     = 1.5     # 移动后等待稳定时间（秒）
TAP_HOLD_TIME      =  1.5    # 点击按住时间（秒）
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
URDF_PATH   = "SO101/so101_5dof_stylus_2.urdf"

# 🔧 测试用硬编码初始关节角度（与 whack_a_mole.py 一致）
INIT_JOINTS = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])


def main():
    print("\n" + "=" * 60)
    print("  单次点击地鼠")
    print("=" * 60)

    camera = None
    robot  = None

    try:
        # ── 1. 相机 ────────────────────────────────────────
        print("\n[1/4] 初始化相机...")
        with open("outputs/cheap_camera_intrinsics.json") as f:
            intrinsics = json.load(f)
        camera_config = OpenCVCameraConfig(
            index_or_path=0, fps=10,
            width=intrinsics["image_width"],
            height=intrinsics["image_height"],
            color_mode=ColorMode.RGB,
        )
        camera = OpenCVCamera(camera_config)
        camera.connect()
        print("   ✅ 相机已连接")

        # ── 2. 检测器 + 坐标转换器 + FK校正 ──────────────────
        print("\n[2/4] 加载标定...")
        detector    = MoleDetector.from_calibration("outputs/mole_calibration.json")
        transformer = CoordinateTransformer()
        table_z = transformer.table_height
        safe_z  = table_z + SAFE_HEIGHT_OFFSET
        tap_z   = table_z + TAP_DEPTH_OFFSET
        print(f"   ✅ 检测器: {len(detector.hole_centers)} 个洞（手动标定）")
        print(f"   ✅ 桌面高度: {table_z * 1000:.1f} mm")

        # FK 校正器
        try:
            fk_corr = FKCorrector.load("outputs/fk_correction.json")
            print("   ✅ FK 校正已加载")
        except FileNotFoundError:
            fk_corr = None
            print("   ℹ️  无 FK 校正文件，使用原始坐标")

        # ── 3. 捕获一帧并检测 ──────────────────────────────
        print("\n[3/4] 捕获画面并检测地鼠...")
        # 先刷新相机缓冲
        for _ in range(5):
            camera.read()
            time.sleep(0.05)

        frame_bgr = capture_frame_bgr(camera)

        # AprilTag 自动定位（若可用，更新洞口像素和深度）
        if _APRILTAG_AVAILABLE:
            try:
                locator = AprilTagLocator.from_intrinsics_file(
                    intrinsics_file="outputs/cheap_camera_intrinsics.json",
                    config_file="outputs/board_config.json",
                )
                if locator.board_data is not None:
                    apriltag_pixels, depth_m = locator.locate(frame_bgr)
                    transformer.update_depth_from_apriltag(depth_m)
                    detector.hole_centers = [(int(u), int(v)) for u, v in apriltag_pixels]
                    detector.monitor_regions = detector._compute_monitor_regions()
                    # 更新 tap_z、safe_z（深度变了，桌面高度也变了）
                    table_z = transformer.table_height
                    safe_z  = table_z + SAFE_HEIGHT_OFFSET
                    tap_z   = table_z + TAP_DEPTH_OFFSET
                    print(f"   ✅ AprilTag 定位成功，深度={depth_m*1000:.0f}mm，"
                          f"桌面高度更新为 {table_z*1000:.1f}mm")
                else:
                    print("   ℹ️  未找到 board_config.json，使用手动标定")
            except FileNotFoundError:
                print("   ℹ️  AprilTag 底板配置不存在，使用手动标定")
            except RuntimeError as e:
                print(f"   ⚠️  AprilTag 定位失败: {e}，使用手动标定")
            except Exception as e:
                print(f"   ⚠️  AprilTag 异常: {e}，使用手动标定")
        else:
            print("   ℹ️  pupil-apriltags 未安装，使用手动标定")

        mole_indices, green_ratios = detector.detect(frame_bgr)
        print(f"   绿色比例: {[f'{r:.0%}' for r in green_ratios]}")

        if not mole_indices:
            print("\n❌ 当前画面未检测到地鼠，退出。")
            return

        # 取第一个检测到的地鼠
        target_idx = mole_indices[0]
        hole_px    = detector.hole_centers[target_idx]
        hole_pos   = transformer.pixel_to_base_3d(hole_px[0], hole_px[1])

        # 应用 FK 逆校正：将真实目标位置转换为 IK 空间目标
        if fk_corr is not None:
            hole_ik = fk_corr.target_for_ik(hole_pos)
            hole_xy = hole_ik[:2]
        else:
            hole_xy = hole_pos[:2]

        print(f"\n🔴 检测到地鼠！洞{target_idx + 1}")
        print(f"   像素坐标: ({hole_px[0]}, {hole_px[1]})")
        print(f"   真实坐标: ({hole_pos[0]*1000:.1f}, {hole_pos[1]*1000:.1f}) mm")
        if fk_corr is not None:
            print(f"   IK目标:   ({hole_xy[0]*1000:.1f}, {hole_xy[1]*1000:.1f}) mm  ← FK校正后")

        # ── 4. 机器人 + 运动学 ─────────────────────────────
        print("\n[4/4] 连接机器人...")
        robot_config = SO101FollowerConfig(
            port="/dev/ttyACM0",
            id="hand_eye_calib_arm",
        )
        robot = SO101Follower(robot_config)
        robot.connect()
        print("   ✅ 机器人已连接")

        # 记录初始关节角度，运行结束后回到此状态
        init_obs = robot.get_observation()
        init_joint_positions = np.array([
            init_obs["shoulder_pan.pos"],
            init_obs["shoulder_lift.pos"],
            init_obs["elbow_flex.pos"],
            init_obs["wrist_flex.pos"],
            init_obs["wrist_roll.pos"],
        ])
        print(f"   📍 初始关节角度: {np.round(init_joint_positions, 2).tolist()}")

        kinematics = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name="stylus_tcp_link",
            joint_names=JOINT_NAMES,
        )
        print("   ✅ 运动学求解器已初始化")

        # ── 5. 点击序列 ────────────────────────────────────
        print("\n" + "=" * 60)
        input("⚠️  按 ENTER 开始点击（Ctrl+C 取消）: ")

        safe_target = np.array([hole_xy[0], hole_xy[1], safe_z])
        tap_target  = np.array([hole_xy[0], hole_xy[1], tap_z])

        # 5a. 移动到洞上方（安全高度）
        safe_joints, error, evec = solve_ik(kinematics, INIT_JOINTS, safe_target)
        print(f"\n→ 移动到洞{target_idx+1}上方 "
              f"(IK误差: {error:.1f} mm  ΔX={evec[0]:+.1f} ΔY={evec[1]:+.1f} ΔZ={evec[2]:+.1f} mm)")
        move_robot(robot, safe_joints)
        time.sleep(MOVE_WAIT_TIME)

        # 5b. 下压点击（从 safe_joints 出发，保持同一解分支，避免 XY 漂移）
        tap_joints, error, evec = solve_ik(kinematics, safe_joints, tap_target)
        print(f"↓ 下压点击 "
              f"(IK误差: {error:.1f} mm  ΔX={evec[0]:+.1f} ΔY={evec[1]:+.1f} ΔZ={evec[2]:+.1f} mm)")
        move_robot(robot, tap_joints)
        time.sleep(TAP_HOLD_TIME)

        # 5c. 抬起（从 tap_joints 出发，平滑返回安全高度）
        lift_joints, error, _ = solve_ik(kinematics, tap_joints, safe_target)
        print("↑ 抬起")
        move_robot(robot, lift_joints)
        time.sleep(MOVE_WAIT_TIME * 0.5)

        # 5d. 回到初始状态
        print("🔙 回到初始状态...")
        move_robot(robot, init_joint_positions)
        time.sleep(MOVE_WAIT_TIME)

        print("\n✅ 点击完成，退出。")

    except KeyboardInterrupt:
        print("\n🛑 用户取消")

    except FileNotFoundError as e:
        print(f"\n❌ 缺少标定文件: {e}")
        print("请先运行: python stage2_test/mole_detector.py --calibrate")

    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        if camera:
            try:
                camera.disconnect()
            except Exception:
                pass
        if robot:
            try:
                robot.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
