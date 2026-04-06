#!/usr/bin/env python3
"""
误差检验 3：舵机重复性误差测试
================================

原理：
  舵机误差包含两部分：
    1. 重复性误差（随机）：每次命令同一角度，实际到达角度有轻微随机偏差。
    2. 系统性偏差（固定）：零位标定偏差、结构形变，导致固定方向偏移。

  测试方法：
    - 让机械臂执行完全相同的关节角命令 N 次（通常 5~10 次）。
    - 在笔尖下方放一张白纸，每次落点用笔轻触留下标记。
    - 观察 N 个落点的散布：
        散布 < 2mm （点集中）  → 重复性好，舵机随机误差小
        散布 > 5mm （点分散）  → 舵机随机误差大
        落点集中但偏离目标     → 是系统性偏差（URDF/手眼标定）而非随机舵机误差

  本脚本额外读取每次执行后的 FK 值，用于辅助分析。

使用方法（从项目根目录运行）:
  python stage2_test/error_analysis/check_servo_error.py

建议准备：
  - 在触控笔下方的桌面放一张白纸用于留下落点印记。
  - 一根细铅笔或马克笔代替触控笔（能清晰留印）效果更好。
"""

import sys
import json
import time
import numpy as np
from pathlib import Path

# 将 stage2_test 加入路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# ─── 配置 ────────────────────────────────────────────────────────────────────
URDF_PATH   = "SO101/so101_5dof_stylus_2.urdf"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ROBOT_PORT  = "/dev/ttyACM0"
ROBOT_ID    = "hand_eye_calib_arm"
OUTPUT_FILE = "outputs/check_servo_error_result.json"

# 重复次数
N_REPEATS = 8

# 每次到达后等待稳定时间（秒）
SETTLE_TIME = 1.5

# 两次测试之间抬起的"中间位置"关节角（避免始终从同一方向接近目标）
LIFT_JOINTS = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])
# ─────────────────────────────────────────────────────────────────────────────


def read_joints(robot) -> np.ndarray:
    obs = robot.get_observation()
    return np.array([obs[f"{n}.pos"] for n in JOINT_NAMES])


def move_and_read(robot, kin, target_joints, settle=SETTLE_TIME):
    """发送关节角命令，等待稳定，读取实际关节角和 FK 位置。"""
    action = {f"{n}.pos": float(target_joints[i]) for i, n in enumerate(JOINT_NAMES)}
    robot.send_action(action)
    time.sleep(settle)
    actual_joints = read_joints(robot)
    T = kin.forward_kinematics(actual_joints)
    fk_pos = T[:3, 3] * 1000  # mm
    return actual_joints, fk_pos


def main():
    print("\n" + "=" * 65)
    print("  误差检验 3：舵机重复性误差测试")
    print("=" * 65)
    print(f"""
说明：
  脚本将把机械臂移动到同一目标关节角度 {N_REPEATS} 次，
  每次移动前会先抬起到中间位置，避免始终从同一方向接近目标。

  你需要：
    1. 在触控笔下方放一张白纸。
    2. 每次机械臂到达目标位置后，用触控笔轻按白纸留下印记。
    3. 测量 {N_REPEATS} 个印记的散布范围（最大间距）。

  如果白纸不方便，可以只看脚本打印的 FK 值的散布。
  （但注意：FK 是"URDF计算值"，不能直接等于实物偏差）
""")

    # ── 选择测试目标关节角 ────────────────────────────────────────────────────
    print("=" * 65)
    print("目标关节角选项：")
    print("  A. 使用预设初始位置（打地鼠初始姿势）")
    print("  B. 使用机械臂当前关节角")
    print("  C. 手动输入关节角")

    while True:
        choice = input("\n请选择 [A/B/C]: ").strip().upper()
        if choice in ("A", "B", "C"):
            break

    print("\n正在连接机器人…")
    robot_config = SO101FollowerConfig(port=ROBOT_PORT, id=ROBOT_ID)
    robot = SO101Follower(robot_config)
    robot.connect()
    print("✅ 机器人已连接\n")

    kin = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name="stylus_tcp_link",
        joint_names=JOINT_NAMES,
    )

    if choice == "A":
        target_joints = LIFT_JOINTS.copy()
        print(f"使用预设关节角: {np.round(target_joints, 2).tolist()}")
    elif choice == "B":
        target_joints = read_joints(robot)
        print(f"使用当前关节角: {np.round(target_joints, 2).tolist()}")
    else:
        print("请依次输入 5 个关节角（度），空格分隔:")
        vals = input("  ").strip().split()
        target_joints = np.array([float(v) for v in vals])
        print(f"使用手动输入关节角: {np.round(target_joints, 2).tolist()}")

    # 先计算目标 FK 位置
    _ = kin.forward_kinematics(target_joints)
    T_target = kin.forward_kinematics(target_joints)
    target_fk_mm = T_target[:3, 3] * 1000
    print(f"\n目标关节角对应的 FK 位置（模型值）:")
    print(f"  X = {target_fk_mm[0]:+.2f} mm")
    print(f"  Y = {target_fk_mm[1]:+.2f} mm")
    print(f"  Z = {target_fk_mm[2]:+.2f} mm")

    print(f"\n准备好白纸后按 Enter 开始 {N_REPEATS} 次重复测试…")
    input()

    results = []

    for i in range(N_REPEATS):
        print(f"\n── 第 {i+1}/{N_REPEATS} 次 " + "─" * 45)

        # 先移动到中间抬起位置（如果不是第一次）
        if i > 0:
            print("   → 先抬起到中间位置…")
            move_and_read(robot, kin, LIFT_JOINTS, settle=1.0)

        # 移动到目标位置
        print("   → 移动到目标位置…")
        actual_joints, fk_pos = move_and_read(robot, kin, target_joints)

        # 关节角偏差
        joint_err = actual_joints - target_joints

        print(f"   实际读取关节角: {np.round(actual_joints, 2).tolist()}")
        print(f"   关节角偏差 (°): {np.round(joint_err, 3).tolist()}")
        print(f"   FK 位置 (mm): X={fk_pos[0]:+.2f}, Y={fk_pos[1]:+.2f}, Z={fk_pos[2]:+.2f}")

        fk_err = fk_pos - target_fk_mm
        print(f"   FK 位置偏差: ΔX={fk_err[0]:+.2f}, ΔY={fk_err[1]:+.2f}, ΔZ={fk_err[2]:+.2f} mm")

        # 提示用手动标记
        if i == 0:
            print("\n   ✋ 请在白纸上标记第 1 个落点（按 Enter 继续）")
        else:
            print(f"\n   ✋ 请标记第 {i+1} 个落点（按 Enter 继续）")
        input()

        results.append({
            "repeat_id": i + 1,
            "target_joints": target_joints.tolist(),
            "actual_joints": actual_joints.tolist(),
            "joint_err_deg": joint_err.tolist(),
            "fk_pos_mm": fk_pos.tolist(),
            "fk_err_mm": fk_err.tolist(),
        })

    # 回到初始位置
    print("\n🔙 测试完毕，回到初始位置…")
    move_and_read(robot, kin, LIFT_JOINTS, settle=1.5)

    try:
        robot.disconnect()
    except Exception:
        pass

    # ── 汇总分析（基于 FK 值的统计）────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("  汇总结果")
    print("=" * 65)

    fk_xs = [r["fk_pos_mm"][0] for r in results]
    fk_ys = [r["fk_pos_mm"][1] for r in results]
    fk_zs = [r["fk_pos_mm"][2] for r in results]

    print(f"\n  重复次数: {N_REPEATS}")
    print(f"\n  FK 位置统计 (mm):")
    print(f"  {'轴':>4}  {'最小':>8}  {'最大':>8}  {'均值':>8}  {'标准差':>8}  {'范围':>8}")
    for axis, vals in [("X", fk_xs), ("Y", fk_ys), ("Z", fk_zs)]:
        mn, mx, mean, std = np.min(vals), np.max(vals), np.mean(vals), np.std(vals)
        rng = mx - mn
        print(f"  {axis:>4}  {mn:>+8.2f}  {mx:>+8.2f}  {mean:>+8.2f}  {std:>8.2f}  {rng:>8.2f}")

    # FK 散布（重复性的模型估计）
    fk_spread = np.sqrt(np.std(fk_xs)**2 + np.std(fk_ys)**2)
    print(f"\n  FK XY 散布 (2D 标准差): {fk_spread:.2f} mm")

    # 关节角重复性
    all_jerr = np.array([r["joint_err_deg"] for r in results])
    print(f"\n  关节角偏差统计 (°):")
    print(f"  {'关节':>12}  {'均值':>8}  {'标准差':>8}")
    for j, name in enumerate(JOINT_NAMES):
        vals = all_jerr[:, j]
        print(f"  {name:>12}  {np.mean(vals):>+8.3f}  {np.std(vals):>8.3f}")

    print("\n" + "─" * 65)
    print("  结论：")
    if fk_spread < 1.0:
        print(f"  ✅  FK 散布 {fk_spread:.2f} mm < 1mm，舵机重复性优秀")
        print("      → 如果实际落点偏差大，主要原因是 URDF 误差或手眼标定误差，而非舵机")
    elif fk_spread < 3.0:
        print(f"  ℹ️  FK 散布 {fk_spread:.2f} mm，舵机重复性一般")
        print("      → 部分误差来自舵机，但可能不是主因")
    else:
        print(f"  ⚠️  FK 散布 {fk_spread:.2f} mm > 3mm，舵机重复性较差")
        print("      → 舵机误差是主要误差来源之一，建议检查舵机零位标定")

    print("\n  📌 补充说明：")
    print("     FK 散布基于 URDF 模型计算，能反映舵机随机误差的相对大小。")
    print("     白纸上的实际落点散布才是真实的舵机重复性指标。")
    print(f"     对比两者：若白纸散布 ≈ FK散布({fk_spread:.1f}mm)，URDF基本准确；")
    print(f"              若白纸散布 >> FK散布({fk_spread:.1f}mm)，URDF 存在较大误差。")

    # 询问白纸上测量的散布
    print("\n  请用尺子测量白纸上所有落点的最大间距（mm）: ", end="")
    spread_str = input().strip()
    if spread_str:
        try:
            real_spread = float(spread_str)
            ratio = real_spread / max(fk_spread, 0.1)
            print(f"\n  白纸散布 = {real_spread:.1f} mm,  FK散布 = {fk_spread:.1f} mm")
            if ratio < 2.0:
                print("  → 两者接近，URDF 对机械误差的建模基本准确")
            else:
                print(f"  → 白纸散布是FK散布的 {ratio:.1f} 倍，URDF 低估了实际舵机误差")
        except ValueError:
            pass

    # 保存结果
    Path("outputs").mkdir(exist_ok=True)
    with open(OUTPUT_FILE, "w") as f:
        json.dump({
            "target_joints": target_joints.tolist(),
            "target_fk_mm": target_fk_mm.tolist(),
            "fk_spread_mm": float(fk_spread),
            "n_repeats": N_REPEATS,
            "results": results,
        }, f, indent=2, ensure_ascii=False)

    print(f"\n  📄 详细结果已保存: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
