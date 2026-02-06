# 🚀 SO101 IK测试 - 快速开始

## ✅ 所有问题已修复！

### 主要修复：
1. ✅ 添加 `SO101FollowerConfig` 配置对象
2. ✅ 使用 `get_observation()` 读取关节位置
3. ✅ 使用正确的 action 格式 `{motor_name}.pos: value`
4. ✅ 设置安全参数 `max_relative_target=10.0`

---

## 🏃 快速运行（3种方式）

### 方式1：使用运行脚本（推荐）✨
```bash
cd /home/zyj/lerobot
./run_ik_test.sh
```

### 方式2：使用conda run
```bash
cd /home/zyj/lerobot
conda run -n lerobot python test_ik_movement.py
```

### 方式3：手动激活环境
```bash
conda activate lerobot
cd /home/zyj/lerobot
python test_ik_movement.py
```

---

## 🔌 指定USB端口（如果不是默认的 /dev/ttyUSB0）

```bash
# 查看可用端口
ls -l /dev/ttyUSB* /dev/ttyACM*

# 使用指定端口运行
./run_ik_test.sh /dev/ttyACM0
# 或
python test_ik_movement.py /dev/ttyACM0
```

---

## 🎮 测试流程建议

### 第一次运行：
1. **选项1** → 查看当前位置
2. **选项3** → 预设位置 → 选择"2"（正前方中位）
3. 观察机械臂移动是否平滑
4. 检查"实际位置误差"是否 < 5mm

### 如果第一次成功：
1. 继续测试其他预设位置（1, 3, 4）
2. 尝试自定义位置（选项2）
3. 记录测试结果

---

## 📊 预设测试位置

```
1. 正前方，低位  (0.25, 0.00, 0.05) ← 模拟手机底部
2. 正前方，中位  (0.30, 0.00, 0.10) ← 模拟手机中心
3. 左前方        (0.25, 0.10, 0.08) ← 模拟手机左侧
4. 右前方        (0.25, -0.10, 0.08) ← 模拟手机右侧
```

---

## ⚠️ 安全检查清单

运行前确认：
- [ ] 机械臂已固定在桌面
- [ ] 运动范围内无障碍物
- [ ] USB线已连接
- [ ] 你已经阅读了完整的 `IK_TEST_GUIDE.md`

---

## 🐛 快速故障排除

### 问题：找不到 lerobot 模块
```bash
# 确保在lerobot环境中运行
conda activate lerobot
```

### 问题：无法连接机械臂
```bash
# 检查USB连接
ls -l /dev/ttyUSB*

# 检查权限
sudo chmod 666 /dev/ttyUSB0
```

### 问题：IK计算失败
- 尝试更近的位置（X < 0.3m）
- 或更远的位置（X > 0.2m）
- 确保Z > 0（不能低于桌面）

---

## 📝 测试成功的标准

如果以下都满足，说明IK功能正常：
- ✅ IK计算误差 < 1mm
- ✅ 实际位置误差 < 5mm
- ✅ 机械臂运动平滑无抖动
- ✅ 可以到达4个预设位置

---

## 🎯 测试成功后的下一步

1. 等待专业标定板到货
2. 进行精确的手眼标定
3. 等待触控笔改造完成
4. 开始开发打地鼠检测算法

---

**准备好了吗？运行 `./run_ik_test.sh` 开始测试！** 🚀
