#!/usr/bin/env python3
"""
URDF/FK 系统误差验证工具
========================

目的：判断 URDF 中 stylus_tcp_link 的几何参数是否与实物一致。

使用方法（从项目根目录运行）:
  python stage1_test/verify_urdf_fk.py

操作步骤:
  1. 运行脚本，机械臂连接后会读取当前关节角并打印 FK 位置。
  2. 在桌面上用马克笔标记几个已知点（或用标定板角点）。
  3. 手动（或用遥控）把触控笔尖精确移到某个标记点上。
  4. 按 Enter 采集：脚本读取关节角、计算 FK，打印触控笔"理论位置"。
  5. 用尺子测量该标记点相对于机械臂基座原点的实际坐标。
  6. 对比 FK 值与实测值，Y 方向偏差如果固定，说明 URDF 是问题所在。

建议至少在 3 个不同位置采集，若误差方向/大小基本一致，则为系统误差。
"""

import numpy as np
import json
import time
from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

URDF_PATH  = "SO101/so101_5dof_stylus_2.urdf"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def read_joints(robot) -> np.ndarray:
    obs = robot.get_observation()
    return np.array([
        obs["shoulder_pan.pos"],
        obs["shoulder_lift.pos"],
        obs["elbow_flex.pos"],
        obs["wrist_flex.pos"],
        obs["wrist_roll.pos"],
    ])


def fk_tip_position(kin: RobotKinematics, joints: np.ndarray) -> np.ndarray:
    """返回 stylus_tcp_link 在基座坐标系下的位置（米）"""
    T = kin.forward_kinematics(joints)
    return T[:3, 3]


def main():
    print("\n" + "=" * 60)
    print("  URDF / FK 系统误差验证工具")
    print("=" * 60)
    print("\n说明：")
    print("  把触控笔尖精确移到桌面已知标记点，按 Enter 采集。")
    print("  脚本输出 FK 计算的触控笔坐标，与你用尺子测量的实际坐标对比。")
    print("  若 Y 方向偏差在各点保持一致，说明 URDF 几何参数有系统误差。\n")

    # ── 连接机器人 ──────────────────────────────────────────
    robot_config = SO101FollowerConfig(
        port="/dev/ttyACM0",
        id="hand_eye_calib_arm",
    )
    robot = SO101Follower(robot_config)
    robot.connect()
    print("✅ 机器人已连接\n")

    # ── 初始化运动学 ────────────────────────────────────────
    kin = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name="stylus_tcp_link",
        joint_names=JOINT_NAMES,
    )
    print("✅ 运动学求解器已初始化\n")

    # 先做一次 FK 预热，避免 placo 内部状态未初始化
    joints_now = read_joints(robot)
    _ = kin.forward_kinematics(joints_now)

    # ── 主采集循环 ──────────────────────────────────────────
    records = []
    print("─" * 60)
    print("操作：")
    print("  1. 禁用力矩后手动把笔尖精确放到标记点")
    print("  2. 输入该标记点的实际坐标（用尺子量，单位 mm，相对基座原点）")
    print("  3. 按 Enter 记录")
    print("  输入 'q' 结束并汇总\n")

    point_id = 1
    while True:
        print(f"\n── 采集点 #{point_id} ──────────────────────────────────")

        # 禁用力矩，让用户手动移动
        try:
            robot.bus.disable_torque()
            print("🔓 力矩已禁用，请手动移动触控笔到标记点...")
        except Exception as e:
            print(f"  (禁用力矩失败: {e}，请手动确保机械臂处于目标位置)")

        cmd = input("\n笔尖已就位后按 Enter 采集，输入 'q' 结束: ").strip().lower()
        if cmd == "q":
            break

        # 启用力矩固定姿态，确保关节角稳定
        try:
            robot.bus.enable_torque()
            time.sleep(0.5)
        except Exception:
            pass

        # 读取关节角 & FK
        joints = read_joints(robot)
        tip_pos = fk_tip_position(kin, joints)

        print(f"\n  关节角 (°): {np.round(joints, 3).tolist()}")
        print(f"  FK 触控笔位置:")
        print(f"    X = {tip_pos[0]*1000:+8.2f} mm")
        print(f"    Y = {tip_pos[1]*1000:+8.2f} mm")
        print(f"    Z = {tip_pos[2]*1000:+8.2f} mm")

        # 让用户输入实测坐标
        print("\n  请用尺子量出该点相对基座原点的实际坐标（mm）：")
        try:
            real_x = float(input("    实际 X (mm): ").strip())
            real_y = float(input("    实际 Y (mm): ").strip())
            real_z = float(input("    实际 Z (mm) [桌面高度输负值，不确定直接回车跳过]: ").strip() or "nan")
        except ValueError:
            print("  输入格式错误，跳过 Z")
            real_z = float("nan")

        fk_mm = tip_pos * 1000
        err_x = fk_mm[0] - real_x
        err_y = fk_mm[1] - real_y
        err_z = fk_mm[2] - real_z if not np.isnan(real_z) else float("nan")

        print(f"\n  ┌─────────────────────────────────────────┐")
        print(f"  │  轴   │  FK值(mm)  │ 实测(mm) │ 误差(mm) │")
        print(f"  ├─────────────────────────────────────────┤")
        print(f"  │  X    │  {fk_mm[0]:+8.2f}  │  {real_x:+7.2f} │  {err_x:+7.2f} │")
        print(f"  │  Y    │  {fk_mm[1]:+8.2f}  │  {real_y:+7.2f} │  {err_y:+7.2f} │")
        if not np.isnan(real_z):
            print(f"  │  Z    │  {fk_mm[2]:+8.2f}  │  {real_z:+7.2f} │  {err_z:+7.2f} │")
        print(f"  └─────────────────────────────────────────┘")

        records.append({
            "point_id": point_id,
            "joints_deg": joints.tolist(),
            "fk_x_mm": float(fk_mm[0]),
            "fk_y_mm": float(fk_mm[1]),
            "fk_z_mm": float(fk_mm[2]),
            "real_x_mm": real_x,
            "real_y_mm": real_y,
            "real_z_mm": real_z,
            "err_x_mm": float(err_x),
            "err_y_mm": float(err_y),
            "err_z_mm": float(err_z) if not np.isnan(err_z) else None,
        })
        point_id += 1

    # ── 汇总分析 ────────────────────────────────────────────
    robot.disconnect()
    print("\n" + "=" * 60)
    print("  汇总结果")
    print("=" * 60)

    if not records:
        print("未采集任何数据，退出。")
        return

    err_x_list = [r["err_x_mm"] for r in records]
    err_y_list = [r["err_y_mm"] for r in records]
    err_z_list = [r["err_z_mm"] for r in records if r["err_z_mm"] is not None]

    print(f"\n  采集点数: {len(records)}")
    print(f"\n  X 方向误差 (FK - 实测):")
    print(f"    均值: {np.mean(err_x_list):+.2f} mm")
    print(f"    标准差: {np.std(err_x_list):.2f} mm")
    print(f"\n  Y 方向误差 (FK - 实测):  ← 重点关注")
    print(f"    均值: {np.mean(err_y_list):+.2f} mm")
    print(f"    标准差: {np.std(err_y_list):.2f} mm")
    if err_z_list:
        print(f"\n  Z 方向误差 (FK - 实测):")
        print(f"    均值: {np.mean(err_z_list):+.2f} mm")
        print(f"    标准差: {np.std(err_z_list):.2f} mm")

    print("\n  各点明细:")
    print(f"  {'点':>4}  {'err_X':>8}  {'err_Y':>8}  {'err_Z':>8}")
    for r in records:
        z_str = f"{r['err_z_mm']:+.2f}" if r["err_z_mm"] is not None else "   N/A"
        print(f"  #{r['point_id']:>3}  {r['err_x_mm']:>+8.2f}  {r['err_y_mm']:>+8.2f}  {z_str:>8}")

    # 判断结论
    print("\n" + "─" * 60)
    y_mean = np.mean(err_y_list)
    y_std  = np.std(err_y_list)
    if abs(y_mean) > 5 and y_std < 5:
        print(f"  ⚠️  结论：Y方向存在系统性偏差 {y_mean:+.1f} mm（标准差 {y_std:.1f} mm）")
        print(f"       → URDF中 stylus_tcp_link 的Y偏移可能需要修正约 {-y_mean:+.1f} mm")
        print(f"       → 或舵机零点在某轴有系统性偏差")
    elif y_std > 8:
        print(f"  ℹ️  结论：Y方向误差随机性较大（标准差 {y_std:.1f} mm），")
        print(f"       → 可能是手动定位精度问题，建议多采集几点并提高定位精度")
    else:
        print(f"  ✅  结论：Y方向误差较小（均值 {y_mean:+.1f} mm），URDF 基本准确")

    # 保存结果
    output_path = "outputs/fk_verification_result.json"
    import json
    with open(output_path, "w") as f:
        json.dump({
            "summary": {
                "n_points": len(records),
                "err_x_mean_mm": float(np.mean(err_x_list)),
                "err_x_std_mm":  float(np.std(err_x_list)),
                "err_y_mean_mm": float(np.mean(err_y_list)),
                "err_y_std_mm":  float(np.std(err_y_list)),
                "err_z_mean_mm": float(np.mean(err_z_list)) if err_z_list else None,
                "err_z_std_mm":  float(np.std(err_z_list))  if err_z_list else None,
            },
            "records": records,
        }, f, indent=2)
    print(f"\n  📄 详细结果已保存: {output_path}")


if __name__ == "__main__":
    main()
