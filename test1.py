import numpy as np
from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# 初始化机器人配置
robot_config = SO101FollowerConfig(
    port="/dev/ttyACM0",
    id="hand_eye_calib_arm",  # 使用你已有的校准配置
)

# 创建并连接机器人
robot = SO101Follower(robot_config)
robot.connect()

# 读取当前关节角度
obs = robot.get_observation()
current_joints = np.array([
    obs["shoulder_pan.pos"],
    obs["shoulder_lift.pos"],
    obs["elbow_flex.pos"],
    obs["wrist_flex.pos"],
    obs["wrist_roll.pos"],
])

print(f"当前关节角度: {current_joints}")

# FK计算gripper_link位置
kin_gripper = RobotKinematics(
    urdf_path="SO101/so101_5dof_stylus.urdf",
    target_frame_name="stylus_tip_link",
    joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
)

pose_gripper = kin_gripper.forward_kinematics(current_joints)
fk_position = pose_gripper[:3, 3] * 1000  # 转换为mm

print(f"\nFK计算的stylus_tip_link位置: [{fk_position[0]:.1f}, {fk_position[1]:.1f}, {fk_position[2]:.1f}] mm")

print("\n" + "="*60)
print("请用尺子测量以下位置（相对于机器人底座）：")
print("  - wrist_roll舵机盘的中心位置")
print("  - X: 从底座中心向前的距离（mm）")
print("  - Y: 从底座中心向右的距离（mm）")
print("  - Z: 从底座底面向上的距离（mm）")
print("="*60)
print("\n将测量结果与FK计算结果对比")
print("如果差异 > 20mm，说明URDF基础部分有问题！")

# 断开连接
robot.disconnect()