#!/usr/bin/env python3
"""
误差检验 2：手眼标定误差验证
==============================

原理：
  手眼标定的作用是建立"相机坐标系"→"机械臂基座坐标系"的变换矩阵
  T_cam_to_base。标定误差包括两类：
    - 平移误差（tx, ty, tz 偏差）：导致所有点向同一方向偏移（常数误差）。
    - 旋转误差（Yaw/Roll/Pitch 偏差）：
        例如 Yaw 偏差会让手机左侧的洞偏左、右侧的洞偏右（位置相关误差）。

本脚本验证方法：
  对手机屏幕上几个已知像素坐标的洞，用代码计算预测的基座坐标，
  然后让机械臂实际移过去，用尺子量实际落点与代码预测位置的差。
  若不同洞的误差方向不一致（例如左洞偏左、右洞偏右）→ 旋转误差。
  若所有洞误差方向和大小一致                          → 平移误差。

前置条件：
  - outputs/cheap_camera_intrinsics.json
  - outputs/camera_to_base_calibration.json
  - outputs/table_height.json
  - outputs/mole_calibration.json（洞的像素坐标）

使用方法（从项目根目录运行）:
  python stage2_test/error_analysis/check_handeye_error.py
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
from coordinate_transformer import CoordinateTransformer

# ─── 配置 ────────────────────────────────────────────────────────────────────
URDF_PATH   = "SO101/so101_5dof_stylus_2.urdf"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ROBOT_PORT  = "/dev/ttyACM0"
ROBOT_ID    = "hand_eye_calib_arm"
OUTPUT_FILE = "outputs/check_handeye_error_result.json"

# IK 初始猜测（用打地鼠时的初始关节角）
INIT_JOINTS = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])

# 安全高度偏移（桌面以上，单位 m）
SAFE_HEIGHT_OFFSET = 0.040
# ─────────────────────────────────────────────────────────────────────────────


def read_joints(robot) -> np.ndarray:
    obs = robot.get_observation()
    return np.array([obs[f"{n}.pos"] for n in JOINT_NAMES])


def solve_ik(kin, current_joints, target_pos):
    """求解 IK，返回关节角和误差（mm）。"""
    target_T = np.eye(4)
    target_T[:3, 3] = target_pos
    _ = kin.forward_kinematics(current_joints)
    q = kin.inverse_kinematics(
        current_joint_pos=current_joints,
        desired_ee_pose=target_T,
        position_weight=1.0,
        orientation_weight=0.01,
    )
    fk_pos = kin.forward_kinematics(q)[:3, 3]
    err_vec = (fk_pos - target_pos) * 1000  # mm
    err_mag = float(np.linalg.norm(err_vec))
    return q, err_mag, err_vec


def move_robot(robot, joints):
    action = {f"{n}.pos": float(joints[i]) for i, n in enumerate(JOINT_NAMES)}
    robot.send_action(action)


def main():
    print("\n" + "=" * 65)
    print("  误差检验 2：手眼标定误差验证")
    print("=" * 65)
    print("""
说明：
  本脚本把机械臂移到各洞的"代码预测位置"，
  让你用尺子量出实际落点与标记点的差距。

  准备工作：
    1. 在手机屏幕/底板上的 9 个洞位置做好物理标记（胶带/马克笔）。
    2. 用尺子预先量好每个洞相对基座原点的真实坐标（可选，更精确）。
       或者目视估计偏差方向（左/右/前/后）和距离。
    3. 建议至少测试 3 个洞：左列、中列、右列各一个。
""")

    # ── 加载坐标转换器（读取手眼标定、内参、桌面高度）────────────────────────
    print("正在加载标定数据…")
    transformer = CoordinateTransformer()
    table_z = transformer.table_height
    safe_z  = table_z + SAFE_HEIGHT_OFFSET
    print(f"  桌面高度: {table_z * 1000:.1f} mm\n")

    # ── 加载洞的像素坐标 ──────────────────────────────────────────────────────
    calib_file = Path("outputs/mole_calibration.json")
    if not calib_file.exists():
        print("❌ 未找到 outputs/mole_calibration.json")
        print("   请先运行: python stage2_test/mole_detector.py --calibrate")
        return

    with open(calib_file) as f:
        mole_calib = json.load(f)

    holes = mole_calib.get("hole_centers", [])
    if not holes:
        print("❌ mole_calibration.json 中没有洞的数据")
        return

    print(f"  共 {len(holes)} 个洞的像素坐标已加载\n")

    # 打印所有洞的像素坐标和预测基座坐标
    print("  洞的像素坐标 → 预测基座坐标（含当前补偿值）：")
    hole_base_coords = []
    for i, (px, py) in enumerate(holes):
        pos = transformer.pixel_to_base_3d(px, py)
        hole_base_coords.append(pos)
        print(f"    洞{i+1}: 像素({px:.0f},{py:.0f}) → 基座({pos[0]*1000:.1f}, {pos[1]*1000:.1f}) mm")

    # ── 连接机器人 ────────────────────────────────────────────────────────────
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
    _ = kin.forward_kinematics(INIT_JOINTS)

    print("─" * 65)
    print("操作说明：")
    print("  脚本会逐个移动机械臂到各洞的代码预测位置（安全高度）。")
    print("  你用尺子测量触控笔尖与该洞标记点的偏差（mm），")
    print("  输入偏差方向+距离，例如：向右 5mm → X方向输入 +5")
    print("  （基座坐标系：X朝前/正面，Y朝左）\n")

    # 让用户选择要测试的洞
    print("  输入要测试的洞编号（1~9，多个用逗号分隔，回车测试所有）: ", end="")
    sel = input().strip()
    if sel:
        indices = [int(x) - 1 for x in sel.split(",") if x.strip().isdigit()]
        indices = [i for i in indices if 0 <= i < len(holes)]
    else:
        indices = list(range(len(holes)))

    print(f"\n将测试: 洞 {[i+1 for i in indices]}\n")
    input("⚠️  按 Enter 开始（Ctrl+C 取消）: ")

    records = []
    current_q = INIT_JOINTS.copy()

    for idx in indices:
        hole_px  = holes[idx]
        hole_pos = hole_base_coords[idx]
        safe_target = np.array([hole_pos[0], hole_pos[1], safe_z])

        print(f"\n── 洞{idx+1} ─────────────────────────────────────────────")
        print(f"   像素: ({hole_px[0]:.0f}, {hole_px[1]:.0f})")
        print(f"   代码预测基座坐标: ({hole_pos[0]*1000:.1f}, {hole_pos[1]*1000:.1f}) mm")
        print(f"   移动到安全高度: Z = {safe_z*1000:.1f} mm")

        # 求解 IK 并移动（安全高度）
        q, err_mag, err_vec = solve_ik(kin, current_q, safe_target)
        print(f"   IK 残差: {err_mag:.1f} mm  "
              f"(ΔX={err_vec[0]:+.1f}, ΔY={err_vec[1]:+.1f}, ΔZ={err_vec[2]:+.1f} mm)")
        move_robot(robot, q)
        current_q = q
        time.sleep(2.0)

        # 让用户测量偏差
        print(f"\n   ✋ 观察触控笔尖与洞{idx+1}标记点的位置关系")
        print("   用尺子量出偏差（触控笔位置 - 标记点位置，单位 mm）：")
        print("   （坐标系：X=基座前方，Y=基座左方；笔在标记点左边则 Y 为正）")
        try:
            dx_str = input("     ΔX (mm) [笔比目标偏前为正，偏后为负]: ").strip()
            dy_str = input("     ΔY (mm) [笔比目标偏左为正，偏右为负]: ").strip()
            dz_str = input("     ΔZ (mm) [安全高度，Z 误差可跳过，回车]: ").strip()

            dx = float(dx_str) if dx_str else float("nan")
            dy = float(dy_str) if dy_str else float("nan")
            dz = float(dz_str) if dz_str else float("nan")
        except ValueError:
            print("   输入格式错误，跳过该点")
            continue

        print(f"\n   洞{idx+1} 实际偏差: ΔX={dx:+.1f} mm, ΔY={dy:+.1f} mm")

        records.append({
            "hole_id": idx + 1,
            "pixel": list(hole_px),
            "predicted_base_mm": (np.array(hole_pos) * 1000).tolist(),
            "physical_error_mm": [dx, dy, dz],
            "ik_residual_mm": err_mag,
            "ik_residual_vec_mm": err_vec.tolist(),
        })

    # 回到初始位置
    print("\n🔙 回到初始位置…")
    q_init, _, _ = solve_ik(kin, current_q, np.array([
        hole_base_coords[0][0], hole_base_coords[0][1], safe_z + 0.05
    ])) if hole_base_coords else (INIT_JOINTS, 0, np.zeros(3))
    move_robot(robot, INIT_JOINTS)
    time.sleep(2.0)

    try:
        robot.disconnect()
    except Exception:
        pass

    # ── 汇总分析 ──────────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("  汇总结果：手眼标定误差分析")
    print("=" * 65)

    if not records:
        print("未采集任何数据，退出。")
        return

    dxs = [r["physical_error_mm"][0] for r in records if not np.isnan(r["physical_error_mm"][0])]
    dys = [r["physical_error_mm"][1] for r in records if not np.isnan(r["physical_error_mm"][1])]

    print(f"\n  采集点数: {len(records)}\n")
    print(f"  {'洞':>4}  {'像素X':>7}  {'像素Y':>7}  "
          f"{'预测X(mm)':>10}  {'预测Y(mm)':>10}  "
          f"{'实物ΔX(mm)':>11}  {'实物ΔY(mm)':>11}")
    for r in records:
        dx_s = f"{r['physical_error_mm'][0]:+.1f}" if not np.isnan(r['physical_error_mm'][0]) else " N/A"
        dy_s = f"{r['physical_error_mm'][1]:+.1f}" if not np.isnan(r['physical_error_mm'][1]) else " N/A"
        print(f"  {r['hole_id']:>4}  "
              f"{r['pixel'][0]:>7.0f}  {r['pixel'][1]:>7.0f}  "
              f"{r['predicted_base_mm'][0]:>10.1f}  {r['predicted_base_mm'][1]:>10.1f}  "
              f"{dx_s:>11}  {dy_s:>11}")

    print("\n" + "─" * 65)

    if dxs:
        print(f"  X 方向物理偏差: 均值={np.mean(dxs):+.1f} mm, 标准差={np.std(dxs):.1f} mm")
    if dys:
        print(f"  Y 方向物理偏差: 均值={np.mean(dys):+.1f} mm, 标准差={np.std(dys):.1f} mm")

    # 自动诊断
    print("\n  诊断：")
    if dys and dxs:
        y_var = np.std(dys)
        x_var = np.std(dxs)
        y_mean = np.mean(dys)
        x_mean = np.mean(dxs)

        # 检查是否是旋转误差（不同洞偏差方向不一致）
        if y_var > 4 or x_var > 4:
            print("  ⚠️  各洞的偏差方向/大小不一致（标准差较大）")
            print("       → 很可能存在手眼标定旋转误差（Yaw偏差）")
            print("       → 建议重新做手眼标定，或对 T_cam_to_base 施加 Yaw 修正")
        elif abs(y_mean) > 5 or abs(x_mean) > 5:
            print(f"  ⚠️  各洞偏差方向一致（均值 X={x_mean:+.1f} mm, Y={y_mean:+.1f} mm）")
            print("       → 是平移型手眼标定误差（tx/ty 偏差）")
            print(f"       → 可在 coordinate_transformer.py 中增加补偿：")
            print(f"          CORRECTION_X += {-x_mean/1000:+.4f} m")
            print(f"          CORRECTION_Y += {-y_mean/1000:+.4f} m")
        else:
            print("  ✅  各洞偏差均较小，手眼标定精度良好")

    # 保存结果
    Path("outputs").mkdir(exist_ok=True)
    with open(OUTPUT_FILE, "w") as f:
        json.dump({
            "summary": {
                "n_points": len(records),
                "dx_mean_mm": float(np.mean(dxs)) if dxs else None,
                "dx_std_mm":  float(np.std(dxs))  if dxs else None,
                "dy_mean_mm": float(np.mean(dys)) if dys else None,
                "dy_std_mm":  float(np.std(dys))  if dys else None,
            },
            "records": records,
        }, f, indent=2, ensure_ascii=False)

    print(f"\n  📄 详细结果已保存: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
