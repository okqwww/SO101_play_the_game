#!/usr/bin/env python3
"""
简单的相机标定脚本 - 获取相机内参（无GUI版本）
"""

import numpy as np
import cv2
import json
from pathlib import Path
import time

def calibrate_camera():
    # 棋盘格参数（与手眼标定使用相同的）
    checkerboard_size = (11, 8)  # 内角点数量
    square_size = 0.005  # 方格大小（米）
    
    # 准备棋盘格世界坐标
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    objpoints = []  # 3D点
    imgpoints = []  # 2D点
    
    # 创建保存目录
    calib_dir = Path("outputs/camera_calibration")
    calib_dir.mkdir(parents=True, exist_ok=True)
    
    # 打开相机
    print("正在打开相机...")
    cap = cv2.VideoCapture(0)  # 修改为你的相机索引
    
    if not cap.isOpened():
        print("❌ 无法打开相机，请检查相机连接")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # 预热相机
    print("相机预热中...")
    for _ in range(10):
        cap.read()
        time.sleep(0.1)
    
    print("\n" + "="*60)
    print("开始采集标定图像（至少需要10张，建议15-20张）")
    print("每次采集会自动检测棋盘格，只有检测成功才会保存")
    print("="*60)
    
    count = 0
    target_count = 15  # 目标采集数量
    
    while count < target_count:
        print(f"\n准备采集第 {count + 1} 张图像...")
        print("请移动棋盘格到不同位置和角度")
        
        
        user_input = input("按回车键拍摄（输入q退出）: ").strip().lower()
        if user_input == 'q':
            break
        
        # 连续读取几帧取最后一帧（确保是最新的）
        for _ in range(5):
            ret, frame = cap.read()
        
        if not ret:
            print("❌ 读取图像失败")
            continue
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 检测棋盘格
        print("  正在检测棋盘格...")
        ret_chess, corners = cv2.findChessboardCorners(
            gray, checkerboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if not ret_chess:
            print("  ❌ 未检测到棋盘格，请调整位置后重试")
            # 保存失败的图片供检查
            fail_path = calib_dir / f"failed_{count:02d}.jpg"
            cv2.imwrite(str(fail_path), frame)
            print(f"  图像已保存到 {fail_path} 供检查")
            continue
        
        # 亚像素精化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # 绘制检测结果
        display = frame.copy()
        cv2.drawChessboardCorners(display, checkerboard_size, corners_refined, ret_chess)
        
        # 保存带标记的图像
        img_path = calib_dir / f"calib_{count:02d}.jpg"
        cv2.imwrite(str(img_path), display)
        
        # 保存原始图像
        raw_path = calib_dir / f"raw_{count:02d}.jpg"
        cv2.imwrite(str(raw_path), frame)
        
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        
        # 保存图像尺寸
        if count == 0:
            img_shape = gray.shape[::-1]  # (width, height)
        
        count += 1
        
        print(f"  ✅ 成功采集第 {count} 张图像")
        print(f"  图像已保存到: {img_path}")
        print(f"  进度: {count}/{target_count}")
    
    cap.release()
    
    print("\n" + "="*60)
    if count < 10:
        print(f"❌ 图像不足，至少需要10张，当前只有{count}张")
        print("请重新运行并采集更多图像")
        return
    
    print(f"✅ 采集完成，共 {count} 张有效图像")
    print("="*60)
    
    # 标定
    print("\n开始标定...")
    
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )
    
    if not ret:
        print("❌ 标定失败")
        return
    
    # 计算重投影误差
    mean_error = 0
    max_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                         camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
        max_error = max(max_error, error)
    
    mean_error /= len(objpoints)
    
    print("\n" + "="*60)
    print("✅ 标定完成！")
    print("="*60)
    print(f"\n重投影误差统计:")
    print(f"  平均误差: {mean_error:.3f} 像素")
    print(f"  最大误差: {max_error:.3f} 像素")
    
    if mean_error > 1.0:
        print(f"\n⚠️  警告：重投影误差较大（>{mean_error:.2f}像素）")
        print("  建议：")
        print("  1. 检查棋盘格是否平整")
        print("  2. 采集更多不同角度的图像")
        print("  3. 确保棋盘格在每张图中清晰可见")
    
    print(f"\n相机内参矩阵:")
    print(camera_matrix)
    print(f"\n畸变系数: {dist_coeffs.flatten()}")
    
    # 保存结果
    calibration_data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.flatten().tolist(),
        "image_width": img_shape[0],
        "image_height": img_shape[1],
        "reprojection_error_mean": float(mean_error),
        "reprojection_error_max": float(max_error),
        "num_images": count
    }
    
    output_file = Path("outputs/cheap_camera_intrinsics.json")
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_file, "w") as f:
        json.dump(calibration_data, f, indent=2)
    
    print(f"\n✅ 内参已保存到: {output_file}")
    print(f"✅ 标定图像已保存到: {calib_dir}")
    print("\n下一步：使用 hand_eye_calibration_opencv.py 进行手眼标定")

if __name__ == "__main__":
    calibrate_camera()