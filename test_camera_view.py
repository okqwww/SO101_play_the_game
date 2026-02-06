#!/usr/bin/env python3
"""
测试相机视角，确保能看到整个手机屏幕
按 'q' 键退出
按 's' 键保存当前帧
"""

import cv2
import numpy as np
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
from lerobot.cameras.configs import ColorMode

def main():
    # 配置相机（替换为你的相机序列号）
    camera_config = RealSenseCameraConfig(
        serial_number_or_name="141722073600",  # 你的相机序列号
        fps=30,
        width=640,
        height=480,
        color_mode=ColorMode.RGB,
    )
    
    # 连接相机
    print("正在连接相机...")
    camera = RealSenseCamera(camera_config)
    camera.connect()
    print("✅ 相机已连接")
    
    print("\n操作提示:")
    print("  - 按 'q' 退出")
    print("  - 按 's' 保存当前画面")
    print("  - 请将手机放在桌面上，打开打地鼠游戏")
    print()
    
    frame_count = 0
    
    try:
        while True:
            # 读取图像
            image = camera.read()
            
            # RGB转BGR（OpenCV显示用）
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # 添加帮助文本
            cv2.putText(image_bgr, "Press 'q' to quit, 's' to save", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 画十字线（帮助对准中心）
            h, w = image_bgr.shape[:2]
            cv2.line(image_bgr, (w//2, 0), (w//2, h), (0, 255, 0), 1)
            cv2.line(image_bgr, (0, h//2), (w, h//2), (0, 255, 0), 1)
            
            # 显示
            cv2.imshow("Camera View - Check Phone Position", image_bgr)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("退出...")
                break
            elif key == ord('s'):
                filename = f"outputs/camera_view_{frame_count:04d}.png"
                cv2.imwrite(filename, image_bgr)
                print(f"✅ 已保存: {filename}")
                frame_count += 1
                
    finally:
        camera.disconnect()
        cv2.destroyAllWindows()
        print("✅ 相机已断开")

if __name__ == "__main__":
    main()