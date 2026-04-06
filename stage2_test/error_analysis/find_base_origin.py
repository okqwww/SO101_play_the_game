#!/usr/bin/env python3
"""
工具：找到机械臂基座坐标系原点的物理位置
==========================================

背景：
  用尺子测量"某点相对于基座原点的坐标"时，必须先知道基座原点在哪里。
  机械臂的基座坐标系原点由 URDF 定义，通常在底座的 base_link 处。
  本脚本通过两种方法帮助你在实物上找到这个原点。

方法一（推荐）：shoulder_pan 旋转弧线法
  - shoulder_pan（第1关节）是纯 Z 轴旋转，其旋转中心就是基座 X=0, Y=0
    在桌面上的投影点。
  - 步骤：
      1. 机械臂保持其他关节角不动，只旋转 shoulder_pan 到多个角度
      2. 每个角度下记录触控笔尖的 (X, Y) 坐标（FK 计算）
      3. 这些点在 XY 平面上构成一段圆弧，圆心即 shoulder_pan 旋转轴，
         也就是基座 (X=0, Y=0) 的投影
      4. 脚本会自动拟合圆弧中心，打印出来

方法二：URDF 直接读取
  - 查看 URDF 的 base_link 在基座坐标系中的原点（始终是 [0,0,0]）。
  - 物理对应：机械臂底座安装螺孔中心，即第一关节旋转轴的交叉点。
  - Z=0 平面：通常是机械臂底座的安装底面（放在桌面上时即桌面附近）。

实际操作建议：
  - 在桌面上把机械臂底座安装位置勾勒出来。
  - 用一根细棍垂直穿过第一关节（shoulder_pan）的旋转轴，
    它指向地面的那个点就是 (X=0, Y=0) 在桌面上的投影。
  - Z 轴朝上，Z=0 在底座底面，用游标卡尺可以量出底座高度。

使用方法（从项目根目录运行）:
  python stage2_test/error_analysis/find_base_origin.py
"""

import sys
import json
import time
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# ─── 配置 ────────────────────────────────────────────────────────────────────
URDF_PATH   = "SO101/so101_5dof_stylus_2.urdf"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ROBOT_PORT  = "/dev/ttyACM0"
ROBOT_ID    = "hand_eye_calib_arm"

# shoulder_pan 的扫描角度（度），其他关节保持当前角度不动
PAN_ANGLES = [-30, -15, 0, 15, 30]   # 单位：度

# 每次转动后等待稳定（秒）
SETTLE_TIME = 1.0
# ─────────────────────────────────────────────────────────────────────────────


def read_joints(robot) -> np.ndarray:
    obs = robot.get_observation()
    return np.array([obs[f"{n}.pos"] for n in JOINT_NAMES])


def fit_circle_2d(points: np.ndarray):
    """
    用最小二乘法拟合 2D 圆弧的圆心。
    points: Nx2 数组，每行是 (x, y)
    返回: (cx, cy, radius)
    """
    # 线性化: x^2+y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)
    # 令 A = [2x, 2y, 1], b = x^2+y^2，最小二乘解 [cx, cy, c3]
    x, y = points[:, 0], points[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x**2 + y**2
    result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = result[0], result[1]
    r = np.sqrt(result[2] + cx**2 + cy**2)
    return cx, cy, r


def method1_sweep(robot, kin):
    """方法一：旋转 shoulder_pan，拟合圆弧中心找基座 XY 原点。"""
    print("\n" + "─" * 65)
    print("  方法一：shoulder_pan 旋转扫描法")
    print("─" * 65)

    # 读取当前关节角，只改第0个关节（shoulder_pan）
    base_joints = read_joints(robot)
    print(f"\n  当前关节角 (°): {np.round(base_joints, 2).tolist()}")
    print(f"  将只旋转 shoulder_pan 到以下角度: {PAN_ANGLES}°")
    print("  其他关节保持不变。\n")
    input("  按 Enter 开始扫描…")

    tip_positions_xy = []

    for angle in PAN_ANGLES:
        joints = base_joints.copy()
        joints[0] = float(angle)  # 只改 shoulder_pan

        # 发送命令
        action = {f"{n}.pos": float(joints[i]) for i, n in enumerate(JOINT_NAMES)}
        robot.send_action(action)
        time.sleep(SETTLE_TIME)

        # 读实际关节角 & FK
        actual = read_joints(robot)
        T = kin.forward_kinematics(actual)
        tip = T[:3, 3] * 1000  # mm

        print(f"  shoulder_pan={angle:+5.1f}° → "
              f"实际={actual[0]:+5.1f}°  |  "
              f"笔尖 (X={tip[0]:+7.1f}, Y={tip[1]:+7.1f}) mm")

        tip_positions_xy.append([tip[0], tip[1]])

    # 拟合圆弧中心
    pts = np.array(tip_positions_xy)
    cx, cy, r = fit_circle_2d(pts)

    print(f"\n  ✅ 拟合结果：")
    print(f"     旋转圆弧半径 r = {r:.1f} mm")
    print(f"     圆心（基座 XY 原点）= (X={cx:+.1f}, Y={cy:+.1f}) mm")
    print(f"\n  ── 物理解读 ──────────────────────────────────────────")
    print(f"  基座坐标系 X=0, Y=0 对应的桌面投影点，")
    print(f"  相对于当前各笔尖落点的圆心在:")
    print(f"    X = {cx:+.1f} mm （正方向 = 机械臂正前方）")
    print(f"    Y = {cy:+.1f} mm （正方向 = 机械臂左侧）")
    print(f"\n  实际操作：")
    print(f"  在桌面上，从触控笔的某个已知落点出发，")
    print(f"  用上面的圆心坐标反推出基座 XY 原点的物理位置。")
    print(f"  例如：若 shoulder_pan=0° 时笔尖在某标记点，")
    print(f"  则基座原点比该标记点 X 方向 {-pts[2][0]+cx:.1f} mm, Y 方向 {-pts[2][1]+cy:.1f} mm。")

    return cx, cy, r


def method2_urdf_info(kin):
    """方法二：直接打印 URDF FK 在零位时的各关节位置。"""
    print("\n" + "─" * 65)
    print("  方法二：URDF 零位 FK 信息")
    print("─" * 65)

    zero_joints = np.zeros(5)
    T = kin.forward_kinematics(zero_joints)
    tip_zero = T[:3, 3] * 1000

    print(f"\n  所有关节为 0° 时（URDF 零位）：")
    print(f"    触控笔尖位置:")
    print(f"      X = {tip_zero[0]:+.2f} mm")
    print(f"      Y = {tip_zero[1]:+.2f} mm")
    print(f"      Z = {tip_zero[2]:+.2f} mm")

    print(f"\n  基座原点定义（URDF base_link）：")
    print(f"    (X=0, Y=0, Z=0)")
    print(f"    → 对应机械臂底座的安装中心（第一关节旋转轴心）")
    print(f"    → Z=0 通常在底座底面（紧贴桌面的位置）")
    print(f"\n  如何在实物上定位：")
    print(f"    1. 找到机械臂底座的安装螺孔圆心")
    print(f"       （或第一关节 shoulder_pan 的旋转轴线）")
    print(f"    2. 该轴线与桌面的交点 = 基座 X=0, Y=0 的桌面投影")
    print(f"    3. Z=0 在底座底面，Z 向上")
    print(f"    4. 机械臂正前方 = X 正方向")
    print(f"    5. 机械臂左侧   = Y 正方向（右手系）")

    print(f"\n  桌面（放手机的表面）相对于基座的 Z 坐标：")
    print(f"    可用 cat outputs/table_height.json 查看")
    try:
        with open("outputs/table_height.json") as f:
            th = json.load(f)
        table_z = th.get("table_height_base", None)
        if table_z is not None:
            print(f"    当前标定值: Z = {table_z * 1000:.1f} mm")
    except Exception:
        print(f"    （table_height.json 未找到）")


def main():
    print("\n" + "=" * 65)
    print("  工具：找到机械臂基座坐标系原点的物理位置")
    print("=" * 65)

    print("""
基座坐标系说明：
  - 原点 (0, 0, 0)：第一关节（shoulder_pan）旋转轴心，在底座底面高度
  - X 轴正方向：机械臂正前方（伸出方向）
  - Y 轴正方向：机械臂左侧（右手坐标系）
  - Z 轴正方向：向上

本脚本提供两种定位方法：
  方法一（推荐）：旋转 shoulder_pan，通过弧线拟合精确确定 XY 原点
  方法二：URDF 零位信息 + 物理操作指导（无需移动）
""")

    choice = input("选择方法 [1=旋转扫描(需连接机器人) / 2=URDF信息(可不连接) / 两者都做=3]: ").strip()

    if choice in ("1", "3"):
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
        _ = kin.forward_kinematics(read_joints(robot))

        method1_sweep(robot, kin)

        if choice == "3":
            method2_urdf_info(kin)

        try:
            robot.disconnect()
        except Exception:
            pass

    elif choice == "2":
        kin = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name="stylus_tcp_link",
            joint_names=JOINT_NAMES,
        )
        method2_urdf_info(kin)

    else:
        print("无效选择，仅显示 URDF 信息（不需要连接机器人）…")
        kin = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name="stylus_tcp_link",
            joint_names=JOINT_NAMES,
        )
        method2_urdf_info(kin)

    print("\n" + "=" * 65)
    print("  测量建议")
    print("=" * 65)
    print("""
实际用尺子测量时的参考流程：
  1. 用本脚本方法一确定基座 XY 原点在桌面上的投影点，用笔标记。
  2. 把 outcomes/table_height.json 中的 Z 值换算为"桌面距底座底面的高度"。
     （Z = table_height_base，即桌面 Z 坐标，正值 = 桌面在底座底面以上）
  3. 测量任意一点时：
     - X 方向：用钢尺从原点标记沿机械臂正前方量
     - Y 方向：从原点标记向机械臂左侧量（正值）或右侧量（负值）
     - Z 方向：从桌面向上量，桌面即为 table_height_base 高度
  4. 注意：所有测量值单位为 mm，输入脚本时为 mm；
     URDF/FK 输出结果是米（m），显示时乘以 1000。
""")


if __name__ == "__main__":
    main()
