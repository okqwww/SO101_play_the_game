import numpy as np
import matplotlib.pyplot as plt
from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

num_trials = 30  # 一共运行30次
dist_fk2target_list = []
ik_error_list = []

# 初始化机器人
robot_config = SO101FollowerConfig(port="/dev/ttyACM0", id="hand_eye_calib_arm")
robot = SO101Follower(robot_config)
robot.connect()

# 关闭伺服，使关节进入松弛模式，可自由手动移动
print("将机械臂切换到自由模式（舵机松弛），方便手动移动...")
robot.bus.disable_torque()
print("已松弛，您现在可以自由移动机械臂。")

# 初始化运动学（使用stylus_tip_link）
kin = RobotKinematics(
    urdf_path="SO101/so101_5dof_stylus.urdf",
    target_frame_name="stylus_tip_link",
    joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
)

# 目标位置（单位：米）
target_position = np.array([0.2306, 0.0452, -0.0079])

print("\n========================================")
print(f"即将进行{num_trials}次IK实验，每次请手动移动机械臂到新位置后回车。")
print("每次会记录两项结果：")
print("  1. 当前末端（FK）与目标的距离（mm）")
print("  2. IK结果的误差（mm）（关节IK后再FK验证的实际误差）")
print("注意：实验完成后会自动画出散点图，横坐标=当前末端与目标距离，纵坐标=IK解的误差")
print("========================================\n")

for trial in range(num_trials):
    print(f"\n========== 第 {trial + 1} / {num_trials} 次 ==========")
    input("请手动移动机械臂到任意姿态，然后按回车...")
    
    # 重新获取一次关节角度（部分舵机会漂移，建议重新上电后采当前位置）
    print("准备采集当前关节角度，需要短暂使能以读取舵机编码器...")
    
    obs = robot.get_observation()
    current_joints = np.array([
        obs["shoulder_pan.pos"],
        obs["shoulder_lift.pos"],
        obs["elbow_flex.pos"],
        obs["wrist_flex.pos"],
        obs["wrist_roll.pos"],
    ])
    
    # 🔧 HARDCODED for testing - ensures identical initial joints as center_reach.py
    # current_joints = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])
    
    # print(f"当前关节角度 (HARDCODED): {current_joints}")
    robot.bus.disable_torque()  # 重新进入松弛

    # 当前末端位置（FK）
    current_pose = kin.forward_kinematics(current_joints)
    current_pos = current_pose[:3, 3]
    fk_mm = current_pos * 1000
    target_mm = target_position * 1000
    dist_fk2target = np.linalg.norm(fk_mm - target_mm)
    print("\n当前关节角度计算出的末端位置(FK): [{:.1f}, {:.1f}, {:.1f}] mm".format(
        fk_mm[0], fk_mm[1], fk_mm[2]
    ))
    print(f"目标位置: {target_mm} mm")
    print(f"当前末端(FK)与目标位置的距离: {dist_fk2target:.2f} mm")
    
    # 构造目标位姿
    target_T = np.eye(4)
    target_T[:3, 3] = target_position

    # IK求解
    print("\n测试1: 纯位置IK（orientation_weight=0）")
    print("-" * 60)
    print("注意注意！")
    print(target_T)
    print(current_joints)
    target_joints = kin.inverse_kinematics(
        current_joint_pos=current_joints,
        desired_ee_pose=target_T,
        position_weight=1.0,
        orientation_weight=0.01
    )
    print(f"IK解出的关节角度: {target_joints}")
    
    # FK验证
    verify_pose = kin.forward_kinematics(target_joints)
    verify_pos = verify_pose[:3, 3]
    error = np.linalg.norm(verify_pos - target_position) * 1000  # mm
    print(f"  FK计算位置: {verify_pos * 1000} mm")
    print(f"\nFK验证:")
    print(f"  目标位置: {target_mm} mm")
    print(f"  FK计算位置: {verify_pos * 1000} mm")
    print(f"  位置误差: {error:.2f} mm")
    
    if error < 10:
        print("\n✅ IK求解成功！误差 < 10mm")
    elif error < 50:
        print("\n⚠️  IK部分成功，误差在可接受范围")
    else:
        print("\n❌ IK求解失败！误差过大")
        print("\n可能原因：")
        print("  1. 目标位置超出工作空间")
        print("  2. 关节限位太严格")
        print("  3. IK求解器配置问题")
    
    dist_fk2target_list.append(dist_fk2target)
    ik_error_list.append(error)

print("\n全部实验完成，开始绘图...\n")
robot.disconnect()

# 只取前20个坐标点用于绘图，如有更多实验
x_plot = dist_fk2target_list
y_plot = ik_error_list

plt.figure(figsize=(8, 6))
plt.scatter(x_plot, y_plot, c='blue', s=60)
plt.xlabel("FK(now) vs target", fontsize=14)
plt.ylabel("IK+FK vs target", fontsize=14)
plt.title("FK-Target distance vs IK error distance", fontsize=16)
plt.grid(True)
plt.tight_layout()
for i, (x, y) in enumerate(zip(x_plot, y_plot)):
    plt.annotate(str(i+1), (x, y), textcoords="offset points", xytext=(0,5), ha='center', fontsize=9, color='gray')
plt.show()


# import numpy as np
# from lerobot.model.kinematics import RobotKinematics
# from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# # 初始化机器人
# robot_config = SO101FollowerConfig(port="/dev/ttyACM0",id="hand_eye_calib_arm")
# robot = SO101Follower(robot_config)
# robot.connect()

# # 读取当前关节角度
# obs = robot.get_observation()
# current_joints = np.array([
#     obs["shoulder_pan.pos"],
#     obs["shoulder_lift.pos"],
#     obs["elbow_flex.pos"],
#     obs["wrist_flex.pos"],
#     obs["wrist_roll.pos"],
# ])

# print(f"当前关节角度: {current_joints}")

# # 初始化运动学（使用stylus_tip_link）
# kin = RobotKinematics(
#     urdf_path="SO101/so101_5dof_stylus.urdf",
#     target_frame_name="stylus_tip_link",
#     joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# )

# # 测试目标：你的实际目标位置（相机中心投影到桌面）
# target_position = np.array([0.2306, 0.0452, -0.0079])  # 单位：米

# print(f"\n目标位置: {target_position * 1000} mm")

# # 构建目标位姿 - 只设置位置，姿态用单位矩阵
# target_T = np.eye(4)
# target_T[:3, 3] = target_position
# # 不设置姿态约束！

# print("\n测试1: 纯位置IK（orientation_weight=0）")
# print("-" * 60)

# # IK求解 - 只约束位置
# target_joints = kin.inverse_kinematics(
#     current_joint_pos=current_joints,
#     desired_ee_pose=target_T,
#     position_weight=1.0,
#     orientation_weight=0.0  # 完全不约束姿态
# )

# print(f"IK解出的关节角度: {target_joints}")

# # 立即FK验证
# verify_pose = kin.forward_kinematics(target_joints)
# verify_pos = verify_pose[:3, 3]
# error = np.linalg.norm(verify_pos - target_position) * 1000

# print(f"\nFK验证:")
# print(f"  目标位置: {target_position * 1000} mm")
# print(f"  FK计算位置: {verify_pos * 1000} mm")
# print(f"  位置误差: {error:.2f} mm")

# if error < 10:
#     print("\n✅ IK求解成功！误差 < 10mm")
# elif error < 50:
#     print("\n⚠️  IK部分成功，误差在可接受范围")
# else:
#     print("\n❌ IK求解失败！误差过大")
#     print("\n可能原因：")
#     print("  1. 目标位置超出工作空间")
#     print("  2. 关节限位太严格")
#     print("  3. IK求解器配置问题")

# print("\n" + "="*60)
# print("现在将舵机松弛，你可以手动移动机械臂到目标位置")
# print("目标位置（相对于底座）:")
# print(f"  X: {target_position[0]*1000:.1f} mm（向前）")
# print(f"  Y: {target_position[1]*1000:.1f} mm（向右）")
# print(f"  Z: {target_position[2]*1000:.1f} mm（高度）")
# print("="*60)

# # 松弛舵机，允许手动移动
# print("\n松弛舵机中...")
# robot.bus.disable_torque()
# print("✅ 舵机已松弛，现在可以手动移动机械臂")

# input("\n手动将触控笔尖移动到目标位置，然后按 ENTER 继续...")

# # 重新启用舵机并读取新位置
# print("\n重新启用舵机...")
# robot.bus.enable_torque()

# # 读取移动后的关节角度
# obs = robot.get_observation()
# manual_joints = np.array([
#     obs["shoulder_pan.pos"],
#     obs["shoulder_lift.pos"],
#     obs["elbow_flex.pos"],
#     obs["wrist_flex.pos"],
#     obs["wrist_roll.pos"],
# ])

# print(f"手动移动后的关节角度: {manual_joints}")

# # FK计算手动移动后的位置
# manual_pose = kin.forward_kinematics(manual_joints)
# manual_pos = manual_pose[:3, 3]

# print(f"\n手动移动后的FK位置: {manual_pos * 1000} mm")
# manual_error = np.linalg.norm(manual_pos - target_position) * 1000
# print(f"与目标位置的误差: {manual_error:.2f} mm")

# print("\n" + "="*60)
# print("总结:")
# print(f"  IK求解误差: {error:.2f} mm")
# print(f"  手动移动误差: {manual_error:.2f} mm")

# if manual_error < 30:
#     print("\n✅ 手动可以到达目标位置（误差<30mm）")
#     if error > 50:
#         print("   → 问题在IK求解器，不是工作空间限制")
#     else:
#         print("   → IK求解器工作正常")
# else:
#     print("\n⚠️  手动也很难精确到达（误差>30mm）")
#     print("   → 可能是目标位置接近工作空间边界")

# print("="*60)

# robot.disconnect()