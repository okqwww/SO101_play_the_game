# 故障排查指南

## 🔧 常见问题及解决方案

### 1. 相机相关问题

#### 问题：AttributeError: 'RealSenseCamera' object has no attribute 'pipeline'

**错误信息：**
```
AttributeError: 'RealSenseCamera' object has no attribute 'pipeline'. 
Did you mean: 'rs_pipeline'?
```

**原因：** RealSenseCamera 类中的 pipeline 属性名为 `rs_pipeline`，不是 `pipeline`。

**解决：** 已在代码中修复，使用 `self.camera.rs_pipeline` 代替 `self.camera.pipeline`。

---

### 2. 机械臂控制问题

#### 问题：机械臂掰不动（力矩未禁用）

**症状：**
- 校准期间可以手动移动机械臂
- 但在"请手动移动机械臂到新位置"提示时掰不动
- 只有 Ctrl+C 中断后才能移动

**原因：** 
机械臂在数据采集循环中力矩保持启用状态，电机锁定了位置。

**解决方案：**
代码已修改为：
1. 提示移动机械臂前 → **禁用力矩**（可以掰动）
2. 按 Enter 准备采集 → **启用力矩**（固定位置，拍照稳定）
3. 采集完成 → 再次禁用力矩（准备下一个位置）

**操作流程：**
```
1. 程序禁用力矩 ✅ ← 现在可以手动移动
2. 手动移动机械臂到新位置
3. 按 Enter
4. 程序启用力矩（固定位置）
5. 拍照采集
6. 返回步骤1（继续下一个位置）
```

---

### 3. URDF文件问题

#### 问题：URDF 文件路径错误

**错误信息：**
```
Error: File SO101/so101_new_calib.urdf/robot.urdf does not exist
ValueError: The file SO101/so101_new_calib.urdf/robot.urdf does not contain a valid URDF model.
```

**解决方案：**

1. **下载正确的URDF文件：**
```bash
cd /home/zyj/lerobot
mkdir -p SO101
cd SO101
wget https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf
```

2. **验证文件存在：**
```bash
ls -lh SO101/so101_new_calib.urdf
# 应该显示一个约10-20KB的文件
```

3. **检查文件内容：**
```bash
head -5 SO101/so101_new_calib.urdf
# 应该看到 <?xml version="1.0"?>
```

---

### 4. 棋盘格检测问题

#### 问题：检测不到棋盘格

**错误信息：**
```
❌ 未检测到棋盘格
```

**可能原因和解决方案：**

| 原因 | 解决方案 |
|------|---------|
| 光线不足 | 增加照明，避免阴影 |
| 光线过强 | 避免强光直射，会产生反光 |
| 棋盘格倾斜太大 | 调整角度，尽量正对相机（< 45°） |
| 棋盘格被遮挡 | 确保完整可见，不被机械臂遮挡 |
| 棋盘格太远或太近 | 调整距离（建议 0.3-0.8m） |
| 棋盘格尺寸设置错误 | 检查代码中的 `checkerboard_size` 参数 |
| 棋盘格打印质量差 | 使用高质量打印，确保黑白分明 |

**调试技巧：**
```bash
# 查看采集的图像
ls -lt outputs/hand_eye_pose_*.png | head -5

# 用图片查看器打开
xdg-open outputs/hand_eye_pose_00.png
```

如果图像中能看到棋盘格但程序检测不到，检查：
- 棋盘格是否平整（不能弯曲）
- 方格是否足够大（至少占视野的 1/4）
- 打印质量是否清晰

---

### 5. 深度信息问题

#### 问题：深度值无效或不准确

**症状：**
```
⚠️ 有效深度点太少: 5/20
❌ 采集失败，请调整位置后重试
```

**可能原因：**
1. **红外干扰**：强光（太阳光、白炽灯）干扰D435i的红外投射
2. **表面材质**：棋盘格表面太亮或反光
3. **距离问题**：太近（< 0.2m）或太远（> 2m）

**解决方案：**

1. **避免红外干扰：**
   - 关闭附近的其他红外设备
   - 避免阳光直射
   - 使用柔和的LED照明

2. **改善棋盘格表面：**
   - 使用哑光纸打印（不要光面纸）
   - 贴在亚克力或硬纸板上（平整）
   - 确保表面不反光

3. **调整距离：**
   ```bash
   # 最佳距离范围：0.3m - 0.8m
   # 太近：深度不准确
   # 太远：角点不清晰
   ```

---

### 6. Conda环境问题

#### 问题：ModuleNotFoundError

**错误信息：**
```
ModuleNotFoundError: No module named 'lerobot'
ModuleNotFoundError: No module named 'cv2'
```

**解决方案：**

1. **确认环境已激活：**
```bash
# 应该看到 (lerobot) 前缀
(lerobot) zyj@zyj:~/lerobot$

# 如果没有，激活环境
conda activate lerobot
```

2. **安装缺失的包：**
```bash
conda activate lerobot

# 基础包
pip install opencv-python scipy

# Lerobot相关
pip install -e ".[feetech]"
pip install -e ".[intelrealsense]"
pip install -e ".[kinematics]"
```

---

### 7. 权限问题

#### 问题：无法访问设备

**错误信息：**
```
Permission denied: /dev/ttyACM0
No RealSense devices detected
```

**解决方案：**

1. **机械臂串口权限：**
```bash
# 临时解决
sudo chmod 666 /dev/ttyACM0

# 永久解决（添加用户到dialout组）
sudo usermod -a -G dialout $USER
# 然后重新登录
```

2. **相机权限：**
```bash
# 检查相机是否被识别
lsusb | grep Intel

# 设置udev规则（已在安装时配置）
ls /etc/udev/rules.d/ | grep realsense
```

---

### 8. 标定精度问题

#### 问题：标定误差很大

**症状：**
```
⚠️  平均误差: 0.015 m (应该 < 0.005m)
```

**可能原因和解决方案：**

| 原因 | 解决方案 |
|------|---------|
| 位姿数量不足 | 采集 15-20 个位姿（而不是 10 个）|
| 位姿分布不均 | 在工作空间各处均匀采集 |
| 棋盘格固定不牢 | 使用更牢固的固定方式，确保不晃动 |
| 方格尺寸测量不准 | 重新精确测量实际方格大小 |
| URDF文件不准确 | 确认使用正确的URDF文件 |
| 深度数据质量差 | 改善光照条件，避免反光 |

**改进步骤：**
1. 删除之前的数据：`rm outputs/hand_eye_data.json`
2. 重新采集，注意位姿分布
3. 采集更多位姿（15-20个）
4. 重新求解标定

---

### 9. 机械臂通信问题

#### 问题：机械臂连接失败

**错误信息：**
```
Unable to open port /dev/ttyACM0
```

**解决步骤：**

1. **检查设备是否连接：**
```bash
ls -l /dev/ttyACM*
# 应该看到 /dev/ttyACM0 或 /dev/ttyACM1
```

2. **如果找不到设备：**
```bash
# 检查USB连接
lsusb | grep -i serial

# 重新插拔USB线
```

3. **如果是 /dev/ttyACM1：**
   修改代码中的端口号：
   ```python
   # hand_eye_calibration.py 第371行
   calibrator = HandEyeCalibrator(
       robot_port="/dev/ttyACM1",  # 修改这里
       ...
   )
   ```

---

### 10. 内存或性能问题

#### 问题：程序运行缓慢或卡顿

**解决方案：**

1. **降低相机分辨率：**
```python
# hand_eye_calibration.py 第72行
camera_config = RealSenseCameraConfig(
    serial_number_or_name=camera_serial,
    fps=30,
    width=640,   # 从1280降到640
    height=480,  # 从720降到480
    ...
)
```

2. **关闭不必要的程序：**
```bash
# 释放内存
free -h

# 关闭其他应用
```

---

## 🆘 获取帮助

如果以上方法都无法解决问题：

1. **查看完整错误信息：**
```bash
python hand_eye_calibration.py 2>&1 | tee error_log.txt
```

2. **检查系统日志：**
```bash
dmesg | tail -50
```

3. **提供以下信息：**
   - 完整的错误信息
   - 运行的命令
   - 系统信息：`uname -a`
   - Python版本：`python --version`
   - 相关文件是否存在

---

## ✅ 快速检查清单

在运行标定前，确认：

- [ ] ✅ Conda环境已激活：`conda activate lerobot`
- [ ] ✅ URDF文件存在：`ls SO101/so101_new_calib.urdf`
- [ ] ✅ 机械臂已连接：`ls /dev/ttyACM*`
- [ ] ✅ 相机已连接：`lerobot-find-cameras realsense`
- [ ] ✅ 棋盘格已固定在机械臂末端
- [ ] ✅ 输出目录存在：`mkdir -p outputs`
- [ ] ✅ 光照条件良好（柔和，无强光）
- [ ] ✅ 工作空间无障碍物

运行检查脚本：
```bash
# TODO: 创建自动检查脚本
./check_system.sh
```
