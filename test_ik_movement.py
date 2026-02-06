#!/usr/bin/env python3
"""
测试SO101机械臂的逆运动学（IK）功能

功能：
1. 获取当前机械臂位置（FK）
2. 设定目标位置（只控制XYZ，姿态自由）
3. 计算IK
4. 移动机械臂到目标位置
5. 验证是否到达
"""

import numpy as np
import time
from pathlib import Path

from lerobot.robots.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics


class IKTester:
    def __init__(self, port: str = "/dev/ttyACM0"):
        print("=" * 60)
        print("SO101 逆运动学（IK）测试程序")
        print("=" * 60)
        
        # 初始化机械臂
        print("\n[1/3] 正在初始化机械臂...")
        print(f"  USB端口: {port}")
        
        # 创建配置
        config = SO101FollowerConfig(
            port=port,
            use_degrees=True,  # 使用度数而不是归一化值
            max_relative_target=None,  # 限制单步最大移动角度（安全）
        )
        
        self.robot = SO101Follower(config)
        print("✅ 机械臂初始化成功！")
        # 连接到机械臂硬件
        print("\n正在连接机械臂...")
        try:
            self.robot.connect(calibrate=True)  # calibrate=False 跳过重新校准
            print("✅ 机械臂连接成功！")
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            print("\n可能的原因：")
            print("  1. USB端口不正确（尝试 /dev/ttyACM0）")
            print("  2. 端口权限不足（运行: sudo chmod 666 /dev/ttyACM0）")
            print("  3. 机械臂电源未开启")
            print("  4. 其他程序正在使用该端口")
            raise
        # 初始化运动学求解器
        print("\n[2/3] 正在初始化运动学求解器...")
        urdf_path = "/home/zyj/lerobot/SO101/so101_new_calib.urdf"
        
        self.kinematics = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name="gripper_frame_link",  # 末端执行器坐标系
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        )
        print("✅ 运动学求解器初始化成功！")
        
        print("\n[3/3] 准备就绪！\n")
    
    def get_current_pose(self):
        """获取当前机械臂位姿"""
        # 读取当前关节角度（观测数据）
        obs = self.robot.get_observation()
        
        # 提取关节位置（按照运动学顺序）
        joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        joint_positions = np.array([obs[f"{name}.pos"] for name in joint_names])
        
        # 正运动学：计算末端位姿
        T = self.kinematics.forward_kinematics(joint_positions)
        
        position = T[:3, 3]  # XYZ位置
        
        return joint_positions, T, position
    
    def move_to_position(self, target_xyz, position_weight=1.0, orientation_weight=0.0):
        """
        移动机械臂到目标位置
        
        Args:
            target_xyz: 目标位置 [x, y, z] (单位: 米)
            position_weight: 位置约束权重（默认1.0）
            orientation_weight: 姿态约束权重（默认0.0，即不约束姿态）
        """
        print("\n" + "=" * 60)
        print(f"目标位置: X={target_xyz[0]:.3f}m, Y={target_xyz[1]:.3f}m, Z={target_xyz[2]:.3f}m")
        print("=" * 60)
        
        # 1. 获取当前状态
        print("\n[步骤1] 读取当前机械臂状态...")
        current_joints, current_T, current_pos = self.get_current_pose()
        print(f"  当前关节角度: {np.round(current_joints, 1)}")
        print(f"  当前末端位置: X={current_pos[0]:.3f}m, Y={current_pos[1]:.3f}m, Z={current_pos[2]:.3f}m")
        
        # 2. 构造目标位姿矩阵（只指定位置，姿态继承当前）
        print(f"\n[步骤2] 构造目标位姿（姿态权重={orientation_weight}）...")
        target_T = current_T.copy()  # 复制当前位姿（保持姿态不变）
        target_T[:3, 3] = target_xyz  # 只修改位置
        
        # 3. 计算IK
        print("\n[步骤3] 计算逆运动学（IK）...")
        try:
            target_joints = self.kinematics.inverse_kinematics(
                current_joint_pos=current_joints,
                desired_ee_pose=target_T,
                position_weight=position_weight,
                orientation_weight=orientation_weight
            )
            print(f"✅ IK计算成功！")
            print(f"  目标关节角度: {np.round(target_joints, 1)}")
            print(f"  关节角度变化: {np.round(target_joints - current_joints, 1)}")
        except Exception as e:
            print(f"❌ IK计算失败: {e}")
            return False
        
        # 4. 验证IK结果（用FK检查）
        print("\n[步骤4] 验证IK结果（正运动学检查）...")
        verify_T = self.kinematics.forward_kinematics(target_joints)
        verify_pos = verify_T[:3, 3]
        ik_error = np.linalg.norm(verify_pos - target_xyz)
        
        print(f"  IK后FK位置: X={verify_pos[0]:.3f}m, Y={verify_pos[1]:.3f}m, Z={verify_pos[2]:.3f}m")
        print(f"  IK计算误差: {ik_error*1000:.2f} mm")
        
        if ik_error > 0.01:  # 误差超过1cm
            print(f"⚠️  警告：IK误差较大（{ik_error*1000:.1f}mm），可能无法准确到达！")
            response = input("  是否仍然继续移动？(y/n): ")
            if response.lower() != 'y':
                print("❌ 用户取消移动")
                return False
        
        # 5. 移动机械臂
        print("\n[步骤5] 发送关节角度命令，移动机械臂...")
        print("⚠️  请确保机械臂周围无障碍物！")
        
        response = input("按 Enter 开始移动，输入 'n' 取消: ")
        if response.lower() == 'n':
            print("❌ 用户取消移动")
            return False
        
        try:
            # 构造动作字典（格式：{motor_name}.pos: value）
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
            action = {f"{name}.pos": target_joints[i] for i, name in enumerate(joint_names)}
            
            self.robot.send_action(action)
            print("✅ 命令已发送！")
            
            # 等待机械臂移动
            print("\n等待机械臂移动到目标位置...")
            time.sleep(3)  # 给机械臂时间移动
            
        except Exception as e:
            print(f"❌ 移动失败: {e}")
            return False
        
        # 6. 验证是否到达
        print("\n[步骤6] 验证机械臂是否到达目标位置...")
        final_joints, final_T, final_pos = self.get_current_pose()
        
        print(f"  实际到达位置: X={final_pos[0]:.3f}m, Y={final_pos[1]:.3f}m, Z={final_pos[2]:.3f}m")
        
        actual_error = np.linalg.norm(final_pos - target_xyz)
        print(f"  实际位置误差: {actual_error*1000:.2f} mm")
        
        if actual_error < 0.01:  # 误差小于1cm
            print("✅ 成功到达目标位置！")
            return True
        elif actual_error < 0.02:  # 误差小于2cm
            print("⚠️  基本到达目标位置（误差可接受）")
            return True
        else:
            print(f"❌ 未能准确到达目标位置（误差 {actual_error*1000:.1f}mm）")
            return False
    
    def interactive_test(self):
        """交互式测试模式"""
        print("\n" + "=" * 60)
        print("交互式IK测试模式")
        print("=" * 60)
        
        # 先显示当前位置
        _, _, current_pos = self.get_current_pose()
        print(f"\n当前末端位置: X={current_pos[0]:.3f}m, Y={current_pos[1]:.3f}m, Z={current_pos[2]:.3f}m")
        
        while True:
            print("\n" + "-" * 60)
            print("请选择操作：")
            print("  1. 查看当前位置")
            print("  2. 移动到指定位置（输入XYZ坐标）")
            print("  3. 预设测试位置")
            print("  4. 退出")
            print("-" * 60)
            
            choice = input("请输入选项 (1-4): ").strip()
            
            if choice == '1':
                _, _, pos = self.get_current_pose()
                print(f"\n当前末端位置: X={pos[0]:.3f}m, Y={pos[1]:.3f}m, Z={pos[2]:.3f}m")
            
            elif choice == '2':
                try:
                    x = float(input("  输入目标X坐标 (米): "))
                    y = float(input("  输入目标Y坐标 (米): "))
                    z = float(input("  输入目标Z坐标 (米): "))
                    
                    target = [x, y, z]
                    self.move_to_position(target)
                except ValueError:
                    print("❌ 输入格式错误，请输入数字！")
            
            elif choice == '3':
                print("\n预设测试位置：")
                print("  1. 正前方，低位  (0.25, 0.00, 0.05)")
                print("  2. 正前方，中位  (0.30, 0.00, 0.10)")
                print("  3. 左前方        (0.25, 0.10, 0.08)")
                print("  4. 右前方        (0.25, -0.10, 0.08)")
                
                preset = input("选择预设位置 (1-4): ").strip()
                
                presets = {
                    '1': [0.25, 0.00, 0.05],
                    '2': [0.30, 0.00, 0.10],
                    '3': [0.25, 0.10, 0.08],
                    '4': [0.25, -0.10, 0.08]
                }
                
                if preset in presets:
                    self.move_to_position(presets[preset])
                else:
                    print("❌ 无效选项！")
            
            elif choice == '4':
                print("\n退出程序...")
                break
            
            else:
                print("❌ 无效选项，请重新选择！")
    
    def cleanup(self):
        """清理资源"""
        print("\n正在断开机械臂连接...")
        # SO101Follower会在析构时自动断开连接
        print("✅ 已断开连接")


def main():
    """主程序"""
    import sys
    
    # 允许通过命令行参数指定端口
    port = "/dev/ttyACM0"  # 默认端口
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    tester = None
    
    try:
        # 创建测试器
        tester = IKTester(port=port)
        
        # 运行交互式测试
        tester.interactive_test()
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断程序（Ctrl+C）")
    
    except Exception as e:
        print(f"\n❌ 程序出错: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if tester is not None:
            tester.cleanup()
        
        print("\n程序结束")


if __name__ == "__main__":
    main()
