# 打地鼠机械臂项目总结

## 🎯 项目目标

使用SO-ARM101机械臂在手机屏幕上自动打地鼠。

## 🔧 硬件配置

- **机械臂**：SO-ARM101 从臂（末端改造为触控橡胶头）
- **相机**：Intel RealSense D435i（固定在桌面正上方）
- **计算平台**：联想笔记本（NVIDIA 3060）
- **控制对象**：手机（平放在桌面）

## 📐 系统架构

```
┌─────────────┐
│  D435i相机   │ (固定在上方)
│  RGB + 深度  │
└──────┬───────┘
       │ 观察
       ▼
┌─────────────┐      识别地鼠位置
│  手机屏幕    │ ◄──── (传统CV检测)
│  打地鼠游戏  │
└──────────────┘
       ▲
       │ 点击
       │
┌──────┴───────┐
│ SO101机械臂   │ (末端橡胶头)
│  自动移动点击  │
└──────────────┘
```

## 🔄 技术流程

### 1. 坐标标定（Eye-to-Hand手眼标定）

**目的**：建立相机坐标系→机械臂基座坐标系的变换关系

```
相机看到的像素 → 相机3D坐标 → 机械臂基座3D坐标 → 末端关节角度
    (x,y,深度)      (x,y,z)_cam     (x,y,z)_base      逆运动学
```

**实现文件**：
- `hand_eye_calibration.py` - 数据采集
- `hand_eye_solver.py` - 求解标定
- `test_calibration.py` - 验证精度

**关键算法**：OpenCV的 `calibrateHandEye()` (Tsai-Lenz等方法)

### 2. 视觉检测

**目的**：实时检测屏幕上的地鼠位置

**方法选项**：
1. **颜色检测**（最简单）- HSV色彩空间阈值分割
2. **模板匹配** - OpenCV `matchTemplate()`
3. **传统机器学习** - HOG+SVM分类器

**输出**：地鼠的3D坐标 (x, y, z)_camera
- 使用像素坐标 + 深度值，充分利用RGBD相机的深度信息
- 通过手眼标定转换到机械臂基座坐标系

### 3. 坐标转换

**使用深度信息**：

```python
# 1. 检测地鼠像素坐标
mole_pixel_x, mole_pixel_y = detect_mole(color_image)

# 2. 获取该位置的深度值（使用周围区域的中位数，更鲁棒）
depth_patch = depth_image[mole_pixel_y-2:mole_pixel_y+3, mole_pixel_x-2:mole_pixel_x+3]
depth = np.median(depth_patch[depth_patch > 0]) * 0.001  # 转换为米

# 3. 像素+深度→相机3D（充分利用RGBD）
point_cam = rs2_deproject_pixel_to_point(intrinsics, [mole_pixel_x, mole_pixel_y], depth)

# 4. 相机3D→基座3D（使用手眼标定结果）
point_base = T_cam_to_base @ [point_cam[0], point_cam[1], point_cam[2], 1]
target_3d = point_base[:3]  # 机械臂可达的3D坐标
```

**实现文件**：`coordinate_transformer.py`

### 4. 轨迹规划

**简单3点轨迹**：

```python
waypoints = [
    [x, y, z + 0.05],  # 悬停在目标上方5cm
    [x, y, z + 0.01],  # 下压到屏幕表面
    [x, y, z + 0.05],  # 抬起
]
```

**不使用MoveIt/ROS**，直接用lerobot的运动学：

```python
from lerobot.model.kinematics import RobotKinematics

# 逆运动学求解
for waypoint in waypoints:
    pose = create_pose_matrix(waypoint)
    joints = kinematics.inverse_kinematics(current_joints, pose)
    robot.send_action(joints)  # 电机自动插值
```

### 5. 主控制循环

```python
while True:
    # 1. 机械臂移到观察位置（不遮挡相机）
    robot.move_to_safe_position()
    
    # 2. 拍照并检测地鼠
    image = camera.read()
    depth_image = camera.read_depth()
    mole_pixel = detect_mole(image)
    
    if mole_pixel:
        # 3. 获取深度
        depth = depth_image[mole_pixel.y, mole_pixel.x]
        
        # 4. 坐标转换
        mole_3d = transformer.pixel_to_base(
            mole_pixel.x, mole_pixel.y, depth, intrinsics
        )
        
        # 5. 规划并执行打击
        waypoints = plan_hit_trajectory(mole_3d)
        execute_trajectory(robot, kinematics, waypoints)
```

## 📁 关键文件

### 已实现的工具

| 文件 | 功能 | 状态 |
|------|------|------|
| `hand_eye_calibration.py` | 手眼标定数据采集 | ✅ |
| `hand_eye_solver.py` | 手眼标定求解 | ✅ |
| `test_calibration.py` | 标定精度验证 | ✅ |
| `coordinate_transformer.py` | 坐标转换工具类 | ✅ |
| `test_camera_view.py` | 相机视角测试 | ✅ |

### 待实现的模块

| 模块 | 优先级 | 预计工作量 |
|------|--------|-----------|
| 地鼠检测器 | 高 | 2-3天 |
| 主控制循环 | 高 | 1-2天 |
| 轨迹优化 | 中 | 1天 |
| 性能调优 | 低 | 1-2天 |

## 🔬 技术细节

### 为什么不用单应矩阵（Homography）？

❌ **之前的方案**：用卷尺测量+单应矩阵（只有2D→2D）
✅ **新方案**：手眼标定+深度信息（完整3D→3D）

**优势**：
1. 精度更高（利用深度信息）
2. 适应性强（手机可以移动）
3. 理论正确（完整的3D变换）

### 为什么不用MoveIt？

- MoveIt需要ROS环境，复杂度高
- 打地鼠场景简单，没有复杂障碍物
- Lerobot已提供足够的功能（逆运动学+插值）

### 插值如何工作？

```
发送: robot.send_action(target_joints)
      ↓
电机内部: 当前角度 ──梯形速度曲线──> 目标角度
         (自动平滑插值，约0.5-1秒)
```

## 🎓 核心知识点

### 1. 坐标变换

```
4x4齐次变换矩阵 T:

┌─────────┬────┐
│  R(3x3) │ t  │  R: 旋转矩阵
├─────────┼────┤  t: 平移向量
│  0 0 0  │ 1  │
└─────────┴────┘

点变换: p' = T @ [p, 1]
```

### 2. 手眼标定方程（Eye-to-hand）

```
AX = XB

A: 机械臂从位置1到位置2的变换
B: 相机观测到标定板从位置1到位置2的变换
X: 要求解的相机到基座的变换
```

### 3. 逆运动学

```
给定: 末端目标位姿 T (4x4矩阵)
求解: 关节角度 θ = [θ1, θ2, θ3, θ4, θ5, θ6]

方法: placo库（基于约束求解）
```

## 📊 性能指标

### 标定精度
- 位置误差：< 5mm
- 旋转误差：< 2°

### 实时性能
- 相机帧率：30 fps
- 检测延迟：< 50ms
- 机械臂响应：< 1秒

### 命中率
- 目标：> 80%（单个地鼠）
- 影响因素：检测精度、运动速度、触控精度

## 🚀 当前进度

✅ **已完成**：
1. 机械臂连接和校准
2. 相机连接和测试
3. 手眼标定代码框架
4. 坐标转换工具

⏳ **进行中**：
1. 准备棋盘格标定板
2. 执行手眼标定

📋 **待完成**：
1. 地鼠检测算法
2. 主控制循环
3. 系统集成测试
4. 性能优化

## 📖 参考资料

1. **Hand-Eye Calibration**:
   - Tsai, R.Y. & Lenz, R.K. (1989). "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration"
   
2. **OpenCV Documentation**:
   - Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
   - Hand-Eye Calibration: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b

3. **RealSense SDK**:
   - Python API: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python

4. **Lerobot Documentation**:
   - GitHub: https://github.com/huggingface/lerobot

## 💡 关键设计决策

| 决策 | 原因 |
|------|------|
| 使用深度相机 | 获得精确的3D信息，避免手动测量 |
| Eye-to-hand标定 | 相机固定更稳定 |
| 不使用VLA/大模型 | 传统CV足够，延迟更低 |
| 不使用MoveIt | 场景简单，Lerobot足够 |
| 检测→执行分离 | 避免遮挡问题 |

---

**项目联系人**：zyj  
**最后更新**：2026-01-15
