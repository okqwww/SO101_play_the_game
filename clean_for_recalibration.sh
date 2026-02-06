#!/bin/bash
# 清理旧数据，准备重新标定

echo "============================================================"
echo "清理旧标定数据"
echo "============================================================"

cd /home/zyj/lerobot

# 创建备份目录
mkdir -p outputs/backup

# 备份旧数据（如果存在）
if [ -f "outputs/hand_eye_data.json" ]; then
    BACKUP_NAME="hand_eye_data_old_$(date +%Y%m%d_%H%M%S).json"
    mv outputs/hand_eye_data.json "outputs/backup/$BACKUP_NAME"
    echo "✅ 已备份: outputs/hand_eye_data.json → outputs/backup/$BACKUP_NAME"
else
    echo "ℹ️  没有找到旧的 hand_eye_data.json"
fi

# 备份旧图片
if ls outputs/hand_eye_pose_*.png 1> /dev/null 2>&1; then
    mv outputs/hand_eye_pose_*.png outputs/backup/
    echo "✅ 已备份: hand_eye_pose_*.png → outputs/backup/"
else
    echo "ℹ️  没有找到旧的采集图片"
fi

# 删除过滤后的数据（会重新生成）
rm -f outputs/hand_eye_data_filtered.json && echo "✅ 已删除: hand_eye_data_filtered.json" || echo "ℹ️  没有 hand_eye_data_filtered.json"
rm -f outputs/camera_to_base_calibration.json && echo "✅ 已删除: camera_to_base_calibration.json" || echo "ℹ️  没有 camera_to_base_calibration.json"
rm -f outputs/camera_to_base_calibration_filtered.json && echo "✅ 已删除: camera_to_base_calibration_filtered.json" || echo "ℹ️  没有 camera_to_base_calibration_filtered.json"

echo ""
echo "============================================================"
echo "✅ 清理完成！现在可以重新标定了"
echo "============================================================"
echo ""
echo "下一步："
echo "  1. conda activate lerobot"
echo "  2. python hand_eye_calibration.py"
echo ""
echo "注意事项："
echo "  - 确保标定板距离相机 > 0.35m"
echo "  - 只保留中心距离误差 < 50mm 的位姿"
echo "  - 采集 20+ 个高质量位姿"
echo ""
