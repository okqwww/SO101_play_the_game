#!/usr/bin/env python3
"""
手动标记手机屏幕四个角点，建立坐标转换关系
使用步骤：
1. 在窗口中点击手机屏幕的四个角（顺序：左上、右上、右下、左下）
2. 程序会保存标定数据到 camera_calibration.json
"""

import cv2
import numpy as np
import json
from pathlib import Path

class CameraPhoneCalibrator:
    def __init__(self, image_path):
        self.image = cv2.imread(str(image_path))
        if self.image is None:
            raise ValueError(f"无法读取图像: {image_path}")
        
        self.display_image = self.image.copy()
        self.points = []
        self.point_labels = ["左上角", "右上角", "右下角", "左下角"]
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            self.points.append((x, y))
            
            # 画点
            cv2.circle(self.display_image, (x, y), 5, (0, 0, 255), -1)
            
            # 画标签
            label = f"{len(self.points)}: {self.point_labels[len(self.points)-1]}"
            cv2.putText(self.display_image, label, (x+10, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 如果已经有2个点，画连线
            if len(self.points) > 1:
                cv2.line(self.display_image, self.points[-2], self.points[-1], 
                        (0, 255, 0), 2)
            
            # 如果4个点都标记了，闭合多边形
            if len(self.points) == 4:
                cv2.line(self.display_image, self.points[3], self.points[0], 
                        (0, 255, 0), 2)
                print("\n✅ 已标记4个角点")
                print("请按 'Enter' 确认，按 'r' 重新标记")
            
            cv2.imshow("标记手机屏幕角点", self.display_image)
    
    def run(self):
        cv2.namedWindow("标记手机屏幕角点")
        cv2.setMouseCallback("标记手机屏幕角点", self.mouse_callback)
        
        print("\n请依次点击手机屏幕的四个角:")
        print("  1. 左上角")
        print("  2. 右上角")
        print("  3. 右下角")
        print("  4. 左下角")
        print("\n完成后按 'Enter' 确认，按 'r' 重新开始\n")
        
        cv2.imshow("标记手机屏幕角点", self.display_image)
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            
            if key == 13 and len(self.points) == 4:  # Enter键
                break
            elif key == ord('r'):  # 重新标记
                self.points = []
                self.display_image = self.image.copy()
                cv2.imshow("标记手机屏幕角点", self.display_image)
                print("重新开始标记...")
            elif key == ord('q'):
                return None
        
        cv2.destroyAllWindows()
        return self.points
    
    def get_world_coordinates(self):
        """让用户输入实际的物理坐标"""
        print("\n现在需要输入这4个角点在真实世界中的坐标（相对于机械臂基座）")
        print("单位：米(m)")
        print("\n请测量并输入以下坐标:")
        print("提示：")
        print("  - X轴：机械臂正前方为正")
        print("  - Y轴：机械臂左侧为正")
        print("  - Z轴：向上为正（手机屏幕表面）\n")
        
        world_points = []
        for i, label in enumerate(self.point_labels):
            print(f"\n{label} (像素坐标: {self.points[i]}):")
            x = float(input(f"  X坐标(米): "))
            y = float(input(f"  Y坐标(米): "))
            z = float(input(f"  Z坐标(米): "))
            world_points.append([x, y, z])
        
        return world_points

def main():
    # 使用之前保存的图像
    image_files = sorted(Path("outputs").glob("camera_view_*.png"))
    
    if not image_files:
        print("❌ 错误：没有找到相机图像")
        print("请先运行 test_camera_view.py 并按 's' 保存图像")
        return
    
    print(f"找到 {len(image_files)} 张图像，使用最新的一张：")
    image_path = image_files[-1]
    print(f"  {image_path}")
    
    # 标记角点
    calibrator = CameraPhoneCalibrator(image_path)
    pixel_points = calibrator.run()
    
    if pixel_points is None:
        print("取消标定")
        return
    
    # 获取世界坐标
    world_points = calibrator.get_world_coordinates()
    
    # 保存标定数据
    calibration_data = {
        "image_path": str(image_path),
        "pixel_points": pixel_points,
        "world_points": world_points,
        "notes": "手机屏幕四个角点：左上、右上、右下、左下"
    }
    
    output_file = "camera_calibration.json"
    with open(output_file, 'w') as f:
        json.dump(calibration_data, f, indent=2)
    
    print(f"\n✅ 标定数据已保存到: {output_file}")
    print("\n标定完成！可以进行下一步了。")

if __name__ == "__main__":
    main()