# 快速开始指南

## 🚀 一键启动

### 方法1：使用启动脚本（推荐）

```bash
cd /home/zyj/lerobot
./RUN_CALIBRATION.sh
```

脚本会自动：
- ✅ 检查并激活conda环境
- ✅ 提供菜单选择操作
- ✅ 检查依赖文件

### 方法2：手动运行

```bash
cd /home/zyj/lerobot

# ⚠️ 第一步：激活环境（非常重要！）
conda activate lerobot

# 第二步：运行对应的脚本
# 数据采集
python hand_eye_calibration.py

# 求解标定
python hand_eye_solver.py

# 验证精度
python test_calibration.py
```

## ⚠️ 重要提醒

### 1. 必须激活conda环境

**在运行任何Python脚本之前，必须先激活lerobot环境：**

```bash
conda activate lerobot
```

验证环境是否激活：
```bash
# 应该看到 (lerobot) 前缀
(lerobot) zyj@zyj:~/lerobot$
```

### 2. 检查依赖

如果遇到导入错误，安装缺失的包：

```bash
# 在lerobot环境中
conda activate lerobot

# 安装依赖
pip install opencv-python scipy
pip install -e ".[feetech]"
pip install -e ".[intelrealsense]"
pip install -e ".[kinematics]"
```

### 3. 检查硬件连接

**运行前确认：**
- [ ] 机械臂已连接（`/dev/ttyACM0`）
- [ ] 相机已连接（D435i）
- [ ] 棋盘格已固定在机械臂末端

**快速测试：**
```bash
# 测试相机
lerobot-find-cameras realsense

# 测试机械臂端口
ls -l /dev/ttyACM*
```

## 📝 完整流程

### 步骤0：准备工作

```bash
# 打印棋盘格（6列×5行，12mm方格）
# 网址：https://calib.io/pages/camera-calibration-pattern-generator
# 设置：Rows=5, Columns=6, Square size=12mm

# 固定棋盘格到机械臂末端
# 确保牢固、平整、不遮挡
```

### 步骤1：数据采集（10-15个位姿）

```bash
conda activate lerobot
./RUN_CALIBRATION.sh
# 选择 1
```

**或手动：**
```bash
conda activate lerobot
python hand_eye_calibration.py
```

**操作：**
1. 手动移动机械臂到不同位置
2. 确保棋盘格在相机视野内
3. 按 Enter 采集
4. 重复10-15次
5. 输入 'q' 退出

### 步骤2：求解标定

```bash
conda activate lerobot
./RUN_CALIBRATION.sh
# 选择 2
```

**或手动：**
```bash
conda activate lerobot
python hand_eye_solver.py
```

**期望输出：**
```
✅ 最佳方法: tsai
   平均误差: 0.003 m

标定结果已保存到: outputs/camera_to_base_calibration.json
```

### 步骤3：验证标定（可选）

```bash
conda activate lerobot
./RUN_CALIBRATION.sh
# 选择 3
```

## 🐛 常见问题

### Q1: ModuleNotFoundError

```
错误：ModuleNotFoundError: No module named 'cv2'
```

**解决：**
```bash
conda activate lerobot
pip install opencv-python
```

### Q2: 相机连接失败

```
错误：No RealSense devices detected
```

**解决：**
```bash
# 检查USB连接（需要USB 3.0）
lerobot-find-cameras realsense

# 检查权限
sudo chmod 666 /dev/video*
```

### Q3: 机械臂端口错误

```
错误：Unable to open port /dev/ttyACM0
```

**解决：**
```bash
# 查找正确的端口
ls -l /dev/ttyACM*

# 修改代码中的端口号
# hand_eye_calibration.py 第371行
```

### Q4: 检测不到棋盘格

```
输出：❌ 未检测到棋盘格
```

**解决：**
- 检查光线（不要太亮或太暗）
- 调整机械臂姿态（减小倾斜）
- 确认棋盘格尺寸设置正确

### Q5: 忘记激活环境

```
错误：No module named 'lerobot'
```

**解决：**
```bash
# 必须先激活环境！
conda activate lerobot

# 然后再运行脚本
python hand_eye_calibration.py
```

## 📂 输出文件

成功完成后，你会得到：

```
outputs/
├── hand_eye_pose_00.png                    # 采集的图像
├── hand_eye_pose_01.png
├── ...
├── hand_eye_data.json                      # 原始数据
├── camera_to_base_calibration.json         # ⭐ 标定结果
└── calibration_validation.json             # 验证结果（可选）
```

**最重要的文件：**
- `camera_to_base_calibration.json` - 后续所有坐标转换都需要这个文件

## ✅ 完成标志

标定成功的标志：
- ✅ 误差 < 5mm
- ✅ 生成了 `camera_to_base_calibration.json`
- ✅ 文件包含 `T_cam_to_base` 矩阵

查看标定结果：
```bash
cat outputs/camera_to_base_calibration.json | python -m json.tool
```

## 🎯 下一步

标定完成后，继续开发：
1. 地鼠检测算法
2. 坐标转换集成
3. 机械臂控制
4. 完整系统测试

---

**遇到问题？** 查看详细文档：
- `cursor_docs/hand_eye_calibration_guide.md` - 完整技术文档
- `cursor_docs/NEXT_STEPS.md` - 详细步骤说明
- `cursor_docs/PROJECT_SUMMARY.md` - 项目总结
