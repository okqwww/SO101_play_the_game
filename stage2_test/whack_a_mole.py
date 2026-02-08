#!/usr/bin/env python3
"""
打地鼠机械臂控制器
==================

主控制循环: 检测地鼠 → 坐标转换 → IK求解 → 机械臂移动 → 抬起 → 循环

使用方法（从项目根目录运行）:
  1. 先完成标定: python stage2_test/mole_detector.py --calibrate
  2. 运行打地鼠: python stage2_test/whack_a_mole.py
  3. 按 Ctrl+C 退出

依赖:
  - outputs/mole_calibration.json  (地鼠检测标定)
  - outputs/cheap_camera_intrinsics.json  (相机内参)
  - outputs/camera_to_base_calibration.json  (手眼标定)
  - outputs/table_height.json  (桌面高度)
"""

import numpy as np
import json
import time
import cv2

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.cameras.configs import ColorMode

from coordinate_transformer import CoordinateTransformer
from mole_detector import MoleDetector

# ═══════════════════════════════════════════════════════════
#  配置参数
# ═══════════════════════════════════════════════════════════

SAFE_HEIGHT_OFFSET = 0.040   # 安全高度：桌面上方 40mm（抬起时）
TAP_DEPTH_OFFSET = 0.002     # 点击深度：桌面以下 2mm（确保接触屏幕）
MOVE_WAIT_TIME = 1.5         # 移动后等待稳定时间（秒）
TAP_HOLD_TIME = 0.3          # 点击按住时间（秒）
DETECTION_INTERVAL = 0.1     # 检测间隔（秒）

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
URDF_PATH = "SO101/so101_5dof_stylus.urdf"


# ═══════════════════════════════════════════════════════════
#  辅助函数
# ═══════════════════════════════════════════════════════════


def read_current_joints(robot) -> np.ndarray:
    """读取机器人当前关节角度。"""
    obs = robot.get_observation()
    return np.array([obs[f"{name}.pos"] for name in JOINT_NAMES])


def solve_ik(kinematics, current_joints, target_position):
    """
    求解IK：给定目标3D位置，返回关节角度。

    关键步骤：先用FK更新placo内部状态（第一阶段发现的关键trick）。

    Args:
        kinematics: RobotKinematics实例
        current_joints: 当前关节角度（度）
        target_position: 目标3D位置（米），[x, y, z]

    Returns:
        target_joints: 目标关节角度（度）
        fk_error_mm: FK验证的位置误差（mm）
    """
    # 构造目标位姿（只约束位置）
    target_T = np.eye(4)
    target_T[:3, 3] = target_position

    # FK预热（必须！更新placo内部状态）
    kinematics.forward_kinematics(current_joints)

    # IK求解
    target_joints = kinematics.inverse_kinematics(
        current_joint_pos=current_joints,
        desired_ee_pose=target_T,
        position_weight=1.0,
        orientation_weight=0.01,
    )

    # FK验证
    verify_pose = kinematics.forward_kinematics(target_joints)
    verify_pos = verify_pose[:3, 3]
    fk_error_mm = np.linalg.norm(verify_pos - target_position) * 1000

    return target_joints, fk_error_mm


def move_robot(robot, target_joints):
    """发送关节角度命令给机器人。"""
    action = {f"{name}.pos": float(target_joints[i]) for i, name in enumerate(JOINT_NAMES)}
    robot.send_action(action)


def capture_frame_bgr(camera) -> np.ndarray:
    """从相机捕获一帧BGR格式图像。"""
    frame_rgb = camera.read()
    if isinstance(frame_rgb, dict):
        frame_rgb = frame_rgb["frame"]
    return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)


# ═══════════════════════════════════════════════════════════
#  主控制循环
# ═══════════════════════════════════════════════════════════


def main():
    print("\n" + "=" * 70)
    print("  打地鼠机械臂控制器")
    print("=" * 70)

    camera = None
    robot = None

    try:
        # ── 1. 初始化所有组件 ─────────────────────────

        # 相机
        print("\n[1/5] 初始化相机...")
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

        # 坐标转换器
        print("\n[2/5] 初始化坐标转换器...")
        transformer = CoordinateTransformer()

        # 地鼠检测器
        print("\n[3/5] 加载地鼠检测标定...")
        detector = MoleDetector.from_calibration("outputs/mole_calibration.json")
        print(f"   ✅ 已加载 {len(detector.hole_centers)} 个洞的标定")

        # 机器人
        print("\n[4/5] 连接机器人...")
        robot_config = SO101FollowerConfig(
            port="/dev/ttyACM0",
            id="hand_eye_calib_arm",
        )
        robot = SO101Follower(robot_config)
        robot.connect()
        print("   ✅ 机器人已连接")

        # 运动学求解器
        print("\n[5/5] 初始化运动学求解器...")
        kinematics = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name="stylus_tip_link",
            joint_names=JOINT_NAMES,
        )
        print("   ✅ 运动学求解器已初始化")

        # ── 2. 预计算安全高度和点击高度 ───────────────

        table_z = transformer.table_height
        safe_z = table_z + SAFE_HEIGHT_OFFSET      # 安全高度（桌面上方）
        tap_z = table_z - TAP_DEPTH_OFFSET          # 点击高度（桌面以下，确保接触）

        print(f"\n📐 高度参数:")
        print(f"   桌面高度: {table_z * 1000:.1f} mm")
        print(f"   安全高度: {safe_z * 1000:.1f} mm (桌面上方 {SAFE_HEIGHT_OFFSET * 1000:.0f} mm)")
        print(f"   点击高度: {tap_z * 1000:.1f} mm (桌面以下 {TAP_DEPTH_OFFSET * 1000:.0f} mm)")

        # ── 3. 预计算9个洞的目标坐标 ─────────────────

        print("\n📍 预计算9个洞的目标坐标:")
        hole_positions = []  # 每个洞在基座坐标系中的 [x, y] (z用安全/点击高度)
        for i, (px, py) in enumerate(detector.hole_centers):
            pos = transformer.pixel_to_base_3d(px, py)
            hole_positions.append(pos[:2])  # 只存 x, y
            print(f"   洞{i + 1}: 像素({px}, {py}) → 基座({pos[0] * 1000:.1f}, {pos[1] * 1000:.1f}) mm")

        # ── 4. 移动到安全位置 ─────────────────────────

        print("\n" + "=" * 70)
        print("  准备就绪")
        print("=" * 70)
        print("\n⚠️  机械臂将在你按回车后开始运行！")
        print("   确保工作空间安全，手机已放好，游戏已开始")
        input("\n按 ENTER 开始打地鼠 (Ctrl+C 退出): ")

        # 先抬到安全高度（用当前位置的x,y）
        current_joints = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])
        current_pose = kinematics.forward_kinematics(current_joints)
        current_xy = current_pose[:3, 3][:2]

        safe_pos = np.array([current_xy[0], current_xy[1], safe_z])
        print(f"\n🔼 抬升到安全高度: Z={safe_z * 1000:.1f} mm")
        target_joints, error = solve_ik(kinematics, current_joints, safe_pos)
        move_robot(robot, target_joints)
        time.sleep(MOVE_WAIT_TIME)
        print("   ✅ 已到达安全高度")

        # ── 5. 主循环 ────────────────────────────────

        print("\n" + "=" * 70)
        print("  🎮 开始打地鼠！按 Ctrl+C 退出")
        print("=" * 70 + "\n")

        total_hits = 0
        last_hit_hole = -1  # 上一次击中的洞（避免重复击打同一个）

        while True:
            # 捕获一帧
            frame_bgr = capture_frame_bgr(camera)

            # 检测地鼠
            mole_indices, green_ratios = detector.detect(frame_bgr)

            if not mole_indices:
                time.sleep(DETECTION_INTERVAL)
                continue

            # 选择目标：优先选不是上次击中的洞
            target_idx = None
            for idx in mole_indices:
                if idx != last_hit_hole:
                    target_idx = idx
                    break
            if target_idx is None:
                target_idx = mole_indices[0]

            hole_px = detector.hole_centers[target_idx]
            hole_xy = hole_positions[target_idx]

            print(f"🔴 检测到地鼠！洞{target_idx + 1} "
                  f"像素({hole_px[0]}, {hole_px[1]}) "
                  f"基座({hole_xy[0] * 1000:.1f}, {hole_xy[1] * 1000:.1f}) mm")

            # ── 移动到目标上方（安全高度）──────────

            current_joints = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])
            safe_target = np.array([hole_xy[0], hole_xy[1], safe_z])
            target_joints, error = solve_ik(kinematics, current_joints, safe_target)

            if error > 20:
                print(f"   ⚠️  IK误差过大 ({error:.1f} mm)，跳过")
                time.sleep(DETECTION_INTERVAL)
                continue

            print(f"   → 移动到洞{target_idx + 1}上方 (IK误差: {error:.1f} mm)")
            move_robot(robot, target_joints)
            time.sleep(MOVE_WAIT_TIME * 0.6)  # 安全高度移动快一点

            # ── 下压点击 ─────────────────────────

            current_joints = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])
            tap_target = np.array([hole_xy[0], hole_xy[1], tap_z])
            target_joints, error = solve_ik(kinematics, current_joints, tap_target)
            print(f"   ↓ 下压点击 (IK误差: {error:.1f} mm)")
            move_robot(robot, target_joints)
            time.sleep(TAP_HOLD_TIME)

            # ── 抬起 ─────────────────────────────

            current_joints = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])
            target_joints, error = solve_ik(kinematics, current_joints, safe_target)
            print(f"   ↑ 抬起")
            move_robot(robot, target_joints)
            time.sleep(MOVE_WAIT_TIME * 0.4)

            total_hits += 1
            last_hit_hole = target_idx
            print(f"   ✅ 击中！总计: {total_hits} 次\n")

    except KeyboardInterrupt:
        print(f"\n\n🛑 用户中断，共击中 {total_hits if 'total_hits' in dir() else 0} 次")

    except FileNotFoundError as e:
        print(f"\n❌ 缺少标定文件: {e}")
        print("请先运行:")
        print("  1. python stage2_test/mole_detector.py --calibrate  (地鼠检测标定)")
        print("  2. 确保 outputs/ 下有相机内参、手眼标定、桌面高度文件")

    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n" + "=" * 70)
        print("  清理")
        print("=" * 70)
        if camera:
            try:
                camera.disconnect()
                print("   ✅ 相机已断开")
            except Exception:
                pass
        if robot:
            try:
                robot.disconnect()
                print("   ✅ 机器人已断开")
            except Exception:
                pass
        print("\n👋 再见!")


if __name__ == "__main__":
    main()
