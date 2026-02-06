# 下一步行动指南

## 📍 当前进度

✅ **已完成：**
- 相机测试（能实时显示手机屏幕）
- 机械臂连接和校准
- 手眼标定代码框架

🎯 **下一步：执行手眼标定**

## 🚀 立即行动

### 第1步：准备棋盘格标定板

**选项A：打印（推荐，快速）**

1. 访问网站生成棋盘格：https://calib.io/pages/camera-calibration-pattern-generator
   - 选择 "Checkerboard"
   - Rows: 5（行数）
   - Columns: 6（列数）
   - **内角点数量：(5, 4)** ← 这是我们在代码中设置的
   - Square size: 12mm（方格大小）
   - 纸张：A4
   - 文件格式：PDF
   - **总尺寸：约 7.2cm × 6cm**（适合机械臂末端）

2. 打印：
   ```bash
   # 确保以实际尺寸打印（不要缩放）
   ```

3. 测量实际方格大小：
   ```bash
   # 用尺子精确测量6个方格的总长度
   # 实际方格大小 = 测量值 / 6
   # 例如：72mm / 6 = 12mm
   ```

4. 固定到硬纸板或亚克力板上（保持平整）

**选项B：购买现成标定板**
- 搜索"OpenCV棋盘格标定板"或"相机标定板"
- 推荐材质：铝板/亚克力（平整度好）
- 记录实际的行列数和方格大小

### 第2步：固定标定板到机械臂末端

```bash
# 临时方案：使用扎带或强力胶带
# 固定要求：
# 1. 标定板牢固，不能晃动
# 2. 标定板平面向外（便于相机观察）
# 3. 不要遮挡机械臂的活动范围
```

**固定位置示意：**
```
机械臂末端（夹爪）
      │
      │  ← 标定板固定在这里
    ┌─┴─┐
    │ ░ │  棋盘格
    │ ░ │  朝向相机
    └───┘
```

### 第3步：修改代码参数（如果需要）

编辑 `hand_eye_calibration.py`：

```python
# 第21行附近，修改棋盘格参数
checkerboard_size: Tuple[int, int] = (9, 6),  # 内角点数量
square_size: float = 0.025,  # 实际测量的方格大小（米）
```

**如何确定内角点数量：**
```
如果你的棋盘格是 10列 x 7行 的方格
那么内角点 = (10-1, 7-1) = (9, 6)
```

### 第4步：执行数据采集

```bash
cd /home/zyj/lerobot

# ⚠️ 重要：先激活conda环境
conda activate lerobot

# 确保已安装依赖
pip install opencv-python scipy

# 运行数据采集
python hand_eye_calibration.py
```

**操作步骤：**

1. 程序启动后，按提示连接硬件
2. 手动移动机械臂到第一个位置：
   - 标定板完整出现在相机视野中
   - 占据视野的 1/4 到 1/3
   - 不要倾斜太大（< 45°）
3. 按 `Enter` 采集
4. 移动到新位置，再次采集
5. **重复10-15次**
6. 输入 `q` 退出

**位姿选择技巧：**
```
相机视野分区（俯视图）：

     远
    ╔═══╗
 左 ║1 2║ 右
    ║3 4║
    ╚═══╝
     近

建议采集顺序：
位姿 1-3:   区域1（左上，不同高度）
位姿 4-6:   区域2（右上，不同高度）
位姿 7-9:   区域3（左下，不同高度）
位姿 10-12: 区域4（右下，不同高度）
位姿 13-15: 中间区域，不同姿态
```

### 第5步：求解标定

```bash
# 确保在lerobot环境中
conda activate lerobot

python hand_eye_solver.py
```

程序会自动：
- 尝试5种算法
- 选择精度最高的
- 保存到 `outputs/camera_to_base_calibration.json`

**期望输出：**
```
✅ 最佳方法: tsai
   平均误差: 0.0032 m  ← 应该 < 0.005m (5mm)
   
标定结果摘要:
  相机相对于机械臂基座的位置:
    X: 0.xxx m
    Y: 0.xxx m
    Z: 0.xxx m  ← 相机在基座上方
```

### 第6步：验证标定（可选但推荐）

```bash
# 确保在lerobot环境中
conda activate lerobot

python test_calibration.py

# 选择测试模式 2
# 输入测试位姿数量: 5
```

**验证标准：**
- 相邻位姿的移动比率应该接近 1.0
- 如果比率在 0.95-1.05 之间，说明标定良好

## 📊 预期输出文件

完成后你会得到：

```
outputs/
├── hand_eye_pose_00.png         # 采集的位姿可视化
├── hand_eye_pose_01.png
├── ...
├── hand_eye_data.json           # 原始标定数据
├── camera_to_base_calibration.json  # ⭐ 最终标定结果
└── calibration_validation.json  # 验证结果（可选）
```

## ⚠️ 常见问题

### Q1: 检测不到棋盘格？
- 检查光线（不要太亮或太暗）
- 调整机械臂姿态（减小倾斜角度）
- 确认棋盘格参数设置正确

### Q2: 误差很大（> 10mm）？
- 增加采集位姿数量（15-20个）
- 确保标定板固定牢靠
- 检查位姿分布是否均匀
- 重新精确测量方格大小

### Q3: 相机启动失败？
```bash
# 检查相机连接
lerobot-find-cameras realsense

# 检查权限
sudo chmod 666 /dev/video*
```

## ✅ 完成检查清单

标定完成后，确认以下文件存在：

```bash
ls -lh outputs/camera_to_base_calibration.json
# 应该显示文件大小约 1-2KB

# 查看标定结果
cat outputs/camera_to_base_calibration.json | python -m json.tool
```

## 🎯 完成后的下一步

标定完成后，你就可以：

1. **使用坐标转换器**：
   ```python
   from coordinate_transformer import CoordinateTransformer
   
   transformer = CoordinateTransformer()
   point_base = transformer.pixel_to_base(pixel_x, pixel_y, depth, intrinsics)
   ```

2. **开发地鼠检测**：
   - 使用OpenCV检测地鼠
   - 获取像素坐标和深度
   - 转换为机械臂坐标
   - 控制机械臂点击

3. **集成完整系统**：
   - 主控制循环
   - 地鼠检测
   - 坐标转换
   - 轨迹规划
   - 机械臂控制

## 📞 需要帮助？

如果遇到问题：
1. 查看 `cursor_docs/hand_eye_calibration_guide.md` 详细文档
2. 检查错误信息
3. 向我描述具体问题和错误输出

---

**现在就开始第1步：准备棋盘格标定板！** 🚀
