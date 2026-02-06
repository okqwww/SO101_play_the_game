# Eye-to-Hand 手眼标定指南

## 📋 任务目标

求解**相机坐标系**到**机械臂基座坐标系**的变换矩阵 `T_cam_to_base`，使得我们可以：
1. 将相机检测到的像素坐标转换为机械臂可以到达的3D坐标
2. 充分利用D435i深度相机的深度信息提高精度

## 🎯 场景说明

- **相机（眼）**：D435i固定在桌面上方（不动）
- **机械臂（手）**：SO101可以移动
- **标定板**：棋盘格固定在机械臂末端

这是典型的 **Eye-to-hand** 手眼标定场景。

## 📐 数学原理

### 坐标系关系

```
机械臂基座 ─── T_base_to_end ──> 机械臂末端 ─── T_end_to_board ──> 标定板
     ↑                                                                  ↑
     |                                                                  |
T_cam_to_base (求解)                                         T_cam_to_board (观测)
     |                                                                  |
   相机 ────────────────────────────────────────────────────────────┘
```

### 手眼标定方程

对于两个不同的机械臂位姿 i 和 j：

```
T_cam_to_base @ T_base_to_end_i @ T_end_to_board = T_cam_to_board_i
T_cam_to_base @ T_base_to_end_j @ T_end_to_board = T_cam_to_board_j
```

消去固定不变的 `T_end_to_board`，得到 **AX = XB** 形式：

```
T_base_to_end_i^(-1) @ T_base_to_end_j = T_cam_to_base^(-1) @ T_cam_to_board_i^(-1) @ T_cam_to_board_j @ T_cam_to_base
```

其中 `X = T_cam_to_base` 是我们要求解的。

## 🛠️ 准备工作

### 1. 制作/购买标定板

**推荐方案A：打印棋盘格（小尺寸）**
- 下载棋盘格模板：https://calib.io/pages/camera-calibration-pattern-generator
- **推荐尺寸**：5×4 内角点（6列×5行格子），每格 12mm
- **总尺寸**：约 7.2cm × 6cm（适合固定在机械臂末端）
- 打印在A4纸上，剪下来贴在硬纸板/亚克力板上（保持平整）
- **重要**：精确测量实际方格大小（可能与设定值略有偏差）

**为什么选择小尺寸？**
- ✅ 易于固定在机械臂末端
- ✅ 相机可以近距离观测，深度精度更高
- ✅ 移动灵活，不易碰撞

**方案B：购买现成标定板**
- 搜索"OpenCV棋盘格标定板"
- 材质：硬质亚克力或铝板（平整度好）
- 尺寸：根据相机视野选择

### 2. 固定标定板到机械臂末端

**固定方式：**
- 使用扎带/胶带固定（临时）
- 3D打印夹具（推荐，稳固）
- 确保标定板固定牢靠，不会晃动

**注意事项：**
- 标定板平面应尽量与末端面平行
- 标定板应突出在末端前方（便于相机观察）
- 确保夹爪不会遮挡标定板

### 3. 下载URDF文件

```bash
# 如果还没有URDF文件
cd /home/zyj/lerobot
mkdir -p SO101
cd SO101
wget https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf
```

## 📊 执行步骤

### 步骤1：数据采集（10-15个位姿）

```bash
cd /home/zyj/lerobot

# ⚠️ 重要：先激活conda环境
conda activate lerobot

python hand_eye_calibration.py
```

**操作流程：**
1. 程序启动后，连接机械臂和相机
2. 手动移动机械臂到第一个位置
   - 确保标定板完整出现在相机视野中
   - 标定板应该占据视野的 1/3 左右
   - 倾斜角度不要太大（< 45°）
3. 按 Enter 采集第一个位姿
4. 移动到新位置，重复采集
5. 采集 10-15 个位姿后，输入 'q' 退出

**位姿选择建议：**
- ✅ 分散在工作空间的不同位置
- ✅ 包含不同的姿态（俯仰、旋转）
- ✅ 距离相机的远近要有变化
- ❌ 避免位姿过于相似
- ❌ 避免标定板被遮挡
- ❌ 避免标定板倾斜过大

**示例位姿分布：**
```
相机视野：

┌─────────────────┐
│   1   2   3     │  ← 上方区域（远）
│                 │
│ 4     5     6   │  ← 中间区域（中）
│                 │
│   7   8   9     │  ← 下方区域（近）
└─────────────────┘

姿态变化：
- 位姿 1,2,3: 机械臂垂直
- 位姿 4,5,6: 机械臂向左/右倾斜
- 位姿 7,8,9: 机械臂向前/后倾斜
```

### 步骤2：求解标定

```bash
# 确保在lerobot环境中
conda activate lerobot

python hand_eye_solver.py
```

程序会：
1. 加载采集的数据
2. 尝试5种不同的求解算法
3. 评估每种算法的精度
4. 自动选择最佳结果并保存

**输出文件：**
- `outputs/camera_to_base_calibration.json` - 标定结果

### 步骤3：验证标定精度

```bash
# 确保在lerobot环境中
conda activate lerobot

python test_calibration.py
```

验证方法：
1. 移动机械臂末端到已知位置
2. 通过正运动学计算末端在基座坐标系的位置
3. 通过相机+标定结果计算末端位置
4. 比较两者差异

## 📈 精度评估

**良好的标定结果：**
- 位置误差 < 5mm
- 旋转误差 < 2°
- 重投影误差 < 3mm

**如果精度不够：**
1. 增加采集位姿数量（15-20个）
2. 提高位姿分布的均匀性
3. 确保标定板固定牢靠
4. 检查URDF文件是否准确
5. 重新精确测量标定板方格大小

## 🔧 故障排查

### 问题1：检测不到棋盘格

**可能原因：**
- 光线不足或过强
- 标定板倾斜角度太大
- 标定板部分被遮挡
- 标定板尺寸设置错误

**解决方案：**
- 调整照明条件
- 调整机械臂姿态
- 确认棋盘格参数（几行几列）

### 问题2：标定误差很大

**可能原因：**
- 位姿数量不足
- 位姿分布不均匀
- 标定板固定不牢（有晃动）
- URDF文件不准确

**解决方案：**
- 增加采集位姿
- 重新采集，注意分布均匀性
- 加固标定板
- 验证URDF文件

### 问题3：深度值不准确

**可能原因：**
- D435i需要预热
- 反光材质干扰
- 深度范围设置不当

**解决方案：**
- 相机启动后等待30秒
- 避免强反光表面
- 检查深度图质量

## 📦 输出文件说明

### hand_eye_data.json
```json
{
  "pose_id": 0,
  "T_base_to_end": [[...]], // 4x4矩阵：基座到末端
  "T_cam_to_board": [[...]], // 4x4矩阵：相机到标定板
  "rvec": [...],             // 旋转向量
  "tvec": [...]              // 平移向量
}
```

### camera_to_base_calibration.json
```json
{
  "T_cam_to_base": [[...]],  // 4x4变换矩阵
  "translation_m": {
    "x": 0.xxx,              // 相机到基座的X距离（米）
    "y": 0.xxx,              // Y距离
    "z": 0.xxx               // Z距离
  },
  "euler_angles_deg": {
    "roll": xx.x,            // 绕X轴旋转
    "pitch": xx.x,           // 绕Y轴旋转
    "yaw": xx.x              // 绕Z轴旋转
  }
}
```

## 🎯 后续使用

标定完成后，可以使用以下函数进行坐标转换：

```python
import numpy as np
import json

# 加载标定结果
with open("outputs/camera_to_base_calibration.json") as f:
    calib = json.load(f)
    T_cam_to_base = np.array(calib["T_cam_to_base"])

# 将相机坐标系中的点转换到机械臂基座坐标系
def camera_to_base(point_cam):
    """
    Args:
        point_cam: [x, y, z] 在相机坐标系中的坐标
    Returns:
        [x, y, z] 在机械臂基座坐标系中的坐标
    """
    point_cam_homo = np.array([*point_cam, 1.0])
    point_base_homo = T_cam_to_base @ point_cam_homo
    return point_base_homo[:3]

# 使用示例
# 1. 从D435i获取3D点（使用深度）
# 2. 转换到基座坐标系
# 3. 控制机械臂移动到该位置
```

## ✅ 检查清单

- [ ] 已制作/购买标定板
- [ ] 标定板固定在机械臂末端
- [ ] 下载SO101的URDF文件
- [ ] 测试相机能看到标定板
- [ ] 采集10-15个不同位姿
- [ ] 运行求解程序
- [ ] 验证标定精度
- [ ] 保存标定结果

## 📚 参考资料

1. Tsai, R. Y., & Lenz, R. K. (1989). A new technique for fully autonomous and efficient 3D robotics hand/eye calibration.
2. OpenCV Hand-Eye Calibration: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
3. RealSense SDK: https://github.com/IntelRealSense/librealsense
