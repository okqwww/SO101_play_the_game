# 测试RealSense D435i是否能被pyrealsense2识别的极简代码
import pyrealsense2 as rs

def check_realsense_device():
    # 初始化RealSense上下文，获取设备列表
    ctx = rs.context()
    device_list = ctx.query_devices()
    
    # 检查是否检测到设备
    if len(device_list) == 0:
        print("❌ 未检测到任何RealSense设备！")
        print("  排查建议：")
        print("  1. 确认D435i插在USB 3.0（蓝色）接口")
        print("  2. 重新插拔摄像头")
        print("  3. 检查udev规则是否生效（重启电脑）")
        return False
    else:
        print(f"✅ 检测到 {len(device_list)} 个RealSense设备：")
        # 打印每个设备的信息
        for idx, dev in enumerate(device_list):
            print(f"\n【设备 {idx+1}】")
            print(f"  设备名称：{dev.get_info(rs.camera_info.name)}")
            print(f"  序列号：{dev.get_info(rs.camera_info.serial_number)}")
            print(f"  固件版本：{dev.get_info(rs.camera_info.firmware_version)}")
        
        # 尝试启动彩色流，验证通信
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            print("\n✅ 成功启动彩色流！摄像头通信正常")
            pipeline.stop()
        except Exception as e:
            print(f"\n⚠️ 启动彩色流失败（不影响基础识别）：{e}")
        return True

if __name__ == "__main__":
    print("=== 开始检测RealSense摄像头 ===")
    check_realsense_device()
    print("=== 检测结束 ===")
