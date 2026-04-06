#!/usr/bin/env python3
"""
舵机标定零位交叉验证
=====================

目的：
  验证舵机标定中的「零位」是否与 URDF 定义的零位在物理上一致。
  这是诊断 FK 误差来源的最关键一步。

原理：
  LeRobot 的角度归一化公式：
    angle_deg = (raw_position - mid) * 360 / 4095
    其中 mid = (range_min + range_max) / 2

  所以「0°」对应的是舵机行程区间的正中点（物理意义上）。
  如果标定时"中间位"的物理姿态与 URDF 的零位不一致，FK 就会出错。

测试步骤：
  === 阶段 1：命令机械臂运动到所有关节 0° ===
  1. 脚本连接机器人，命令所有关节运动到 0°
  2. 等待机械臂到位
  3. 你对照 URDF 可视化（所有 joint 设为 0°）观察物理姿态是否匹配
  4. 脚本打印实际读回的关节角度（应接近 0°）

  === 阶段 2：逐关节运动测试（验证方向和量级）===
  对 shoulder_pan / shoulder_lift / elbow_flex 逐一进行：
  5. 将关节命令到 +30°，等到位
  6. 对照可视化（该关节设为 +30°），检查方向是否一致
  7. 再命令到 -30°，同样检查

使用方法（从项目根目录运行）：
  python stage2_test/error_analysis/check_calib_zero.py
"""

import sys
import time
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# ─── 配置 ────────────────────────────────────────────────────────────────────
ROBOT_PORT  = "/dev/ttyACM0"
ROBOT_ID    = "hand_eye_calib_arm"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 每次移动后等待到位的时间（秒）
MOVE_WAIT = 3.0
# ─────────────────────────────────────────────────────────────────────────────


def read_joints(robot) -> np.ndarray:
    obs = robot.get_observation()
    return np.array([obs[f"{n}.pos"] for n in JOINT_NAMES])


def move_to(robot, angles_deg: list[float], wait: float = MOVE_WAIT):
    """命令机械臂移动到指定关节角度（度）并等待"""
    action = {f"{n}.pos": float(angles_deg[i]) for i, n in enumerate(JOINT_NAMES)}
    robot.send_action(action)
    time.sleep(wait)


def print_joints(robot, label: str = ""):
    joints = read_joints(robot)
    prefix = f"[{label}] " if label else ""
    print(f"  {prefix}实际关节角 (°):")
    for name, val in zip(JOINT_NAMES, joints):
        print(f"    {name:15s}: {val:+7.2f}°")
    return joints


def main():
    print("\n" + "=" * 65)
    print("  舵机标定零位交叉验证")
    print("=" * 65)
    print("""
说明：
  本脚本命令机械臂运动到各种已知角度，你对照 URDF 可视化工具
  （把对应 joint 滑块设为相同值）检查物理姿态是否匹配。

  如果物理实物和 URDF 可视化的姿态不一致 → 说明标定零位有偏差！
""")

    # ── 连接机器人 ──────────────────────────────────────────────────────────
    print("正在连接机器人…")
    robot_config = SO101FollowerConfig(port=ROBOT_PORT, id=ROBOT_ID)
    robot = SO101Follower(robot_config)
    robot.connect()
    print("✅ 机器人已连接\n")

    # ── 显示当前标定信息 ────────────────────────────────────────────────────
    calib = robot.bus.calibration
    print("─" * 65)
    print("当前标定数据（用于计算角度零位）：")
    print(f"  角度公式：angle = (raw_pos - mid) × 360 / 4095")
    print(f"  其中 mid = (range_min + range_max) / 2\n")
    print(f"  {'关节':<18} {'range_min':>10} {'range_max':>10} {'mid':>10}  (mid=0°处)")
    print("  " + "-" * 55)
    for name in JOINT_NAMES:
        c = calib[name]
        mid = (c.range_min + c.range_max) / 2
        print(f"  {name:<18} {c.range_min:>10} {c.range_max:>10} {mid:>10.1f}")
    print()

    # ── 读取当前关节角 ──────────────────────────────────────────────────────
    print("─" * 65)
    print("机械臂当前状态：")
    print_joints(robot, "当前")

    input("\n⚠️  确保机械臂周围空间安全后按 Enter，脚本将命令机械臂运动…")

    # =========================================================================
    # 阶段 1：命令所有关节到 0°
    # =========================================================================
    print("\n" + "─" * 65)
    print("【阶段 1】命令所有关节到 0°")
    print(f"  正在移动（等待 {MOVE_WAIT}s）…")
    move_to(robot, [0, 0, 0, 0, 0])
    actual = print_joints(robot, "到位后")

    print("\n  ✅ 请打开 URDF 可视化工具（如 RViz 或 Cursor URDF 插件），")
    print("     将 so101_5dof_stylus_2.urdf 所有 joint 滑块设为 0°。")
    print("     观察：物理实物 vs URDF 可视化 是否姿态一致？")
    print("\n  期望偏差：< 5°  ← 说明标定基本准确")
    print("  实际偏差：")
    for name, val in zip(JOINT_NAMES, actual):
        status = "✅" if abs(val) < 5 else "⚠️ " if abs(val) < 15 else "❌"
        print(f"    {status} {name:<15}: 当前读数 {val:+.2f}°（期望 0°，偏差 {abs(val):.2f}°）")

    input("\n对照完成后按 Enter 继续下一阶段…")

    # =========================================================================
    # 阶段 2：逐关节单独测试（shoulder_pan / shoulder_lift / elbow_flex）
    # =========================================================================
    test_joints = [
        ("shoulder_pan",   "绕机械臂竖直轴左右旋转（机械臂底座旋转）"),
        ("shoulder_lift",  "整个大臂上下摆动（第2关节）"),
        ("elbow_flex",     "小臂相对大臂弯曲（第3关节）"),
    ]

    for joint_name, description in test_joints:
        print("\n" + "─" * 65)
        print(f"【阶段 2 - {joint_name}】")
        print(f"  关节描述：{description}")

        # 先回到全零位
        print(f"  回到零位（等待 {MOVE_WAIT}s）…")
        move_to(robot, [0, 0, 0, 0, 0])

        # +30°
        target = [0, 0, 0, 0, 0]
        idx = JOINT_NAMES.index(joint_name)
        target[idx] = 30.0
        print(f"\n  命令 {joint_name} → +30°（等待 {MOVE_WAIT}s）…")
        move_to(robot, target)
        actual_plus = print_joints(robot, "+30° 到位")

        print(f"\n  ✅ 请在 URDF 可视化中把 {joint_name} 设为 +30°，")
        print(f"     其余关节保持 0°。")
        print(f"     观察：物理旋转方向 是否与可视化一致？")
        answer_plus = input(f"  方向是否一致？(y/n/s=跳过) ").strip().lower()

        # -30°
        target[idx] = -30.0
        print(f"\n  命令 {joint_name} → -30°（等待 {MOVE_WAIT}s）…")
        move_to(robot, target)
        actual_minus = print_joints(robot, "-30° 到位")

        print(f"\n  ✅ 请在 URDF 可视化中把 {joint_name} 设为 -30°。")
        answer_minus = input(f"  方向是否一致？(y/n/s=跳过) ").strip().lower()

        if answer_plus == 'n' or answer_minus == 'n':
            print(f"\n  ⚠️  {joint_name} 的方向或角度有问题！")
            print(f"     可能原因：drive_mode 符号反转，或 homing_offset 设置有误。")

    # =========================================================================
    # 阶段 3：FK 零位验证（不连 kinematics，直接打印标定中位）
    # =========================================================================
    print("\n" + "─" * 65)
    print("【阶段 3】回到全零位 + FK 计算（需要 Placo 库）")

    try:
        from lerobot.model.kinematics import RobotKinematics
        kin = RobotKinematics(
            urdf_path="SO101/so101_5dof_stylus_2.urdf",
            target_frame_name="stylus_tcp_link",
            joint_names=JOINT_NAMES,
        )
        _ = kin.forward_kinematics(np.zeros(5))

        print(f"  正在移动到全零位（等待 {MOVE_WAIT}s）…")
        move_to(robot, [0, 0, 0, 0, 0])
        joints_at_zero = read_joints(robot)
        T_at_zero = kin.forward_kinematics(joints_at_zero)
        tip_zero = T_at_zero[:3, 3] * 1000

        T_ideal = kin.forward_kinematics(np.zeros(5))
        tip_ideal = T_ideal[:3, 3] * 1000

        print(f"\n  全零位时 FK 结果：")
        print(f"    理论零位 (关节全0°) FK:  X={tip_ideal[0]:+.1f}  Y={tip_ideal[1]:+.1f}  Z={tip_ideal[2]:+.1f} mm")
        print(f"    实际到位后读回的 FK:      X={tip_zero[0]:+.1f}  Y={tip_zero[1]:+.1f}  Z={tip_zero[2]:+.1f} mm")
        diff = tip_zero - tip_ideal
        print(f"    两者差：                  ΔX={diff[0]:+.1f}  ΔY={diff[1]:+.1f}  ΔZ={diff[2]:+.1f} mm")
        if np.linalg.norm(diff) < 5:
            print(f"    ✅ FK 在零位附近一致，运动学计算正常")
        else:
            print(f"    ⚠️  FK 零位差 {np.linalg.norm(diff):.1f}mm，可能有标定偏差")

    except ImportError:
        print("  (Placo 库未安装，跳过 FK 计算)")

    # ── 回到安全位置并断开 ──────────────────────────────────────────────────
    print("\n" + "─" * 65)
    input("测试完成，按 Enter 让机械臂回到零位并断开…")
    print(f"  回到零位（等待 {MOVE_WAIT}s）…")
    move_to(robot, [0, 0, 0, 0, 0])

    try:
        robot.disconnect()
    except Exception:
        pass

    print("\n✅ 测试完成！\n")
    print("分析建议：")
    print("  1. 若零位姿态与 URDF 可视化不一致 → 需重新标定（注意「中间位」的物理定义）")
    print("  2. 若 +30° / -30° 方向错误 → 标定中的 drive_mode 或 homing 方向有误")
    print("  3. 若零位姿态一致但 check_urdf_error.py 仍有大误差 → 测量基座原点有误")
    print()


if __name__ == "__main__":
    main()
