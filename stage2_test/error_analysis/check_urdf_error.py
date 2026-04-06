#!/usr/bin/env python3
"""
误差检验 1：URDF / FK 系统误差验证
====================================

原理：
  - 正运动学 (FK) 完全依赖 URDF 中的连杆参数（长度、偏移、扭转角）。
  - 如果 URDF 几何与实物不一致，FK 算出的末端位置就会与真实位置存在
    **固定方向、固定大小** 的系统偏差。
  - 舵机误差是随机/小幅的；URDF 误差是跨点一致的。

操作步骤：
  1. 在桌面上用马克笔或胶带标记 3~5 个点（建议覆盖机械臂工作空间）。
  2. 用尺子量出每个标记点相对机械臂基座原点的实际坐标（mm）。
     （如何找基座原点 → 运行 find_base_origin.py）
  3. 运行本脚本，它会禁用力矩，让你手动把触控笔尖移到标记点。
  4. 就位后按 Enter，脚本读取关节角并用 FK 计算笔尖坐标。
  5. 脚本对比 FK 值与你输入的实测值，打印误差。
  6. 采集完所有点后，脚本汇总分析：
     - 误差方向/大小 **在各点基本一致** → URDF 有系统误差
     - 误差 **随机/较小**               → URDF 基本准确，舵机可能是主因

使用方法（从项目根目录运行）:
  python stage2_test/error_analysis/check_urdf_error.py
"""

import sys
import json
import time
import numpy as np
from pathlib import Path

# 将 stage2_test 加入路径（以便导入 FKCorrector 等）
sys.path.insert(0, str(Path(__file__).parent.parent))

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# FK 校正（如果存在校正文件）
FK_CORRECTION_FILE = "outputs/fk_correction.json"

# ─── 配置 ────────────────────────────────────────────────────────────────────
URDF_PATH   = "SO101/so101_5dof_stylus_2.urdf"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ROBOT_PORT  = "/dev/ttyACM0"
ROBOT_ID    = "hand_eye_calib_arm"
OUTPUT_FILE = "outputs/check_urdf_error_result.json"
# ─────────────────────────────────────────────────────────────────────────────


def read_joints(robot) -> np.ndarray:
    obs = robot.get_observation()
    return np.array([obs[f"{n}.pos"] for n in JOINT_NAMES])


def main():
    print("\n" + "=" * 65)
    print("  误差检验 1：URDF / FK 系统误差验证")
    print("=" * 65)
    print("""
说明：
  把触控笔尖精确放到桌面已知标记点，按 Enter 采集。
  脚本会打印 FK 计算位置与你实测位置的误差。
  若多点误差方向/大小一致 → URDF 参数有问题。
  若误差随机或很小         → URDF 基本准确。

  建议至少采集 3 个不同位置的点，
  涵盖机械臂前方、左侧、右侧各一个。
""")

    # ── 连接机器人 ────────────────────────────────────────────────────────────
    print("正在连接机器人…")
    robot_config = SO101FollowerConfig(port=ROBOT_PORT, id=ROBOT_ID)
    robot = SO101Follower(robot_config)
    robot.connect()
    print("✅ 机器人已连接\n")

    # ── 初始化运动学 ──────────────────────────────────────────────────────────
    kin = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name="stylus_tcp_link",
        joint_names=JOINT_NAMES,
    )
    # Placo 预热（避免第一次 FK 内部状态未初始化）
    _ = kin.forward_kinematics(read_joints(robot))
    print("✅ 运动学求解器已初始化\n")

    # ── 加载 FK 校正（如果存在）────────────────────────────────────────────
    fk_corrector = None
    try:
        from fk_corrector import FKCorrector
        fk_corrector = FKCorrector.load(FK_CORRECTION_FILE)
        print("✅ 已加载 FK 校正参数\n")
    except (FileNotFoundError, ImportError):
        print("ℹ️  未找到 FK 校正文件，显示原始 FK 结果")
        print("   （如需校正：先采集数据，再运行 calibrate_fk.py）\n")

    print("─" * 65)
    print("操作提示：")
    print("  1. 禁用力矩后，手动将触控笔尖精确放到桌面标记点。")
    print("  2. 输入该标记点的实测坐标（单位 mm，相对基座原点）。")
    print("     （不知道基座原点在哪？先运行 find_base_origin.py）")
    print("  3. 按 Enter 记录。输入 'q' 结束并查看汇总。\n")

    records = []
    point_id = 1

    while True:
        print(f"\n── 采集点 #{point_id} " + "─" * 40)

        # 禁用力矩，让用户手动移动
        try:
            robot.bus.disable_torque()
            print("🔓 力矩已禁用，请手动移动触控笔到标记点…")
        except Exception as e:
            print(f"  (禁用力矩失败: {e}，请手动将机械臂移到目标位置)")

        cmd = input("\n笔尖已就位后按 Enter 采集，输入 'q' 结束: ").strip().lower()
        if cmd == "q":
            break

        # 启用力矩固定姿态，读取关节角
        try:
            robot.bus.enable_torque()
            time.sleep(0.5)
        except Exception:
            pass

        joints = read_joints(robot)
        T = kin.forward_kinematics(joints)
        tip_raw = T[:3, 3] * 1000  # 转换为 mm

        # 如果有校正，计算校正后的值
        if fk_corrector is not None:
            tip_corrected_m = fk_corrector.correct_fk(T[:3, 3])
            tip = tip_corrected_m * 1000
        else:
            tip = tip_raw

        print(f"\n  关节角 (°): {np.round(joints, 2).tolist()}")
        if fk_corrector is not None:
            print(f"  FK 原始位置:   X={tip_raw[0]:+8.2f}  Y={tip_raw[1]:+8.2f}  Z={tip_raw[2]:+8.2f} mm")
            print(f"  FK 校正后位置: X={tip[0]:+8.2f}  Y={tip[1]:+8.2f}  Z={tip[2]:+8.2f} mm  ← 用这个")
        else:
            print(f"  FK 触控笔位置:")
            print(f"    X = {tip[0]:+8.2f} mm")
            print(f"    Y = {tip[1]:+8.2f} mm")
            print(f"    Z = {tip[2]:+8.2f} mm")

        # 输入实测坐标
        print("\n  请用尺子量出该点相对基座原点的实际坐标 (mm)：")
        try:
            rx = float(input("    实际 X (mm): ").strip())
            ry = float(input("    实际 Y (mm): ").strip())
            rz_str = input("    实际 Z (mm) [不确定直接回车跳过]: ").strip()
            rz = float(rz_str) if rz_str else float("nan")
        except ValueError:
            print("  输入格式错误，该点跳过。")
            continue

        ex = tip[0] - rx
        ey = tip[1] - ry
        ez = tip[2] - rz if not np.isnan(rz) else float("nan")

        print(f"\n  ┌──────────────────────────────────────────────┐")
        print(f"  │  轴  │  FK值(mm)  │  实测(mm)  │  误差(mm)  │")
        print(f"  ├──────────────────────────────────────────────┤")
        print(f"  │  X   │  {tip[0]:+9.2f} │  {rx:+9.2f} │  {ex:+9.2f} │")
        print(f"  │  Y   │  {tip[1]:+9.2f} │  {ry:+9.2f} │  {ey:+9.2f} │")
        if not np.isnan(rz):
            print(f"  │  Z   │  {tip[2]:+9.2f} │  {rz:+9.2f} │  {ez:+9.2f} │")
        print(f"  └──────────────────────────────────────────────┘")

        records.append({
            "point_id": point_id,
            "joints_deg": joints.tolist(),
            "fk_mm": tip.tolist(),
            "real_mm": [rx, ry, rz],
            "err_mm": [float(ex), float(ey), float(ez) if not np.isnan(ez) else None],
        })
        point_id += 1

    # ── 断开机器人 ────────────────────────────────────────────────────────────
    try:
        robot.disconnect()
    except Exception:
        pass

    # ── 汇总分析 ──────────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("  汇总结果")
    print("=" * 65)

    if not records:
        print("未采集任何数据，退出。")
        return

    exs = [r["err_mm"][0] for r in records]
    eys = [r["err_mm"][1] for r in records]
    ezs = [r["err_mm"][2] for r in records if r["err_mm"][2] is not None]

    print(f"\n  采集点数: {len(records)}")

    for axis, vals, label in [("X", exs, ""), ("Y", eys, "  ← 重点关注"), ("Z", ezs, "")]:
        if not vals:
            continue
        mean = np.mean(vals)
        std  = np.std(vals)
        print(f"\n  {axis} 方向误差 (FK - 实测){label}:")
        print(f"    均值    = {mean:+.2f} mm")
        print(f"    标准差  = {std:.2f} mm")

    print("\n  各点明细:")
    print(f"  {'点':>4}  {'err_X':>9}  {'err_Y':>9}  {'err_Z':>9}")
    for r in records:
        ez_str = f"{r['err_mm'][2]:+.2f}" if r["err_mm"][2] is not None else "   N/A"
        print(f"  #{r['point_id']:>3}  {r['err_mm'][0]:>+9.2f}  {r['err_mm'][1]:>+9.2f}  {ez_str:>9}")

    print("\n" + "─" * 65)

    # 自动结论
    ey_mean = np.mean(eys)
    ey_std  = np.std(eys)
    ex_mean = np.mean(exs)
    ex_std  = np.std(exs)

    issues = []
    for axis, mean, std, name in [("X", ex_mean, ex_std, "X"), ("Y", ey_mean, ey_std, "Y")]:
        if abs(mean) > 5 and std < 5:
            issues.append(
                f"  ⚠️  {name} 方向系统性偏差 {mean:+.1f} mm（标准差 {std:.1f} mm）\n"
                f"       → URDF 中该方向参数或舵机零点可能需要修正约 {-mean:+.1f} mm"
            )
        elif std > 8:
            issues.append(
                f"  ℹ️  {name} 方向误差随机性较大（标准差 {std:.1f} mm）\n"
                f"       → 可能是手动定位精度不足，建议提高定位精度后重测"
            )
        else:
            issues.append(f"  ✅  {name} 方向误差较小（均值 {mean:+.1f} mm），该方向 URDF 基本准确")

    for msg in issues:
        print(msg)

    # 保存结果
    Path("outputs").mkdir(exist_ok=True)
    with open(OUTPUT_FILE, "w") as f:
        json.dump({
            "summary": {
                "n_points": len(records),
                "err_x_mean_mm": float(np.mean(exs)),
                "err_x_std_mm":  float(np.std(exs)),
                "err_y_mean_mm": float(np.mean(eys)),
                "err_y_std_mm":  float(np.std(eys)),
                "err_z_mean_mm": float(np.mean(ezs)) if ezs else None,
                "err_z_std_mm":  float(np.std(ezs))  if ezs else None,
            },
            "records": records,
        }, f, indent=2, ensure_ascii=False)

    print(f"\n  📄 详细结果已保存: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
