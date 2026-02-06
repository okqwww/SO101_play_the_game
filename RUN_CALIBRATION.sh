#!/bin/bash
# 手眼标定快速启动脚本

echo "======================================"
echo "  手眼标定 - 快速启动"
echo "======================================"

# 检查conda环境
if [ -z "$CONDA_DEFAULT_ENV" ] || [ "$CONDA_DEFAULT_ENV" != "lerobot" ]; then
    echo ""
    echo "⚠️  检测到未激活lerobot环境"
    echo "正在激活 conda 环境..."
    eval "$(conda shell.bash hook)"
    conda activate lerobot
    
    if [ $? -ne 0 ]; then
        echo "❌ 激活失败，请手动运行："
        echo "   conda activate lerobot"
        exit 1
    fi
fi

echo ""
echo "✅ Conda环境: $CONDA_DEFAULT_ENV"
echo ""

# 菜单选择
echo "请选择操作："
echo "  1. 数据采集（第一步）"
echo "  2. 求解标定（第二步）"
echo "  3. 验证标定（第三步）"
echo "  4. 退出"
echo ""

read -p "请输入选项 (1-4): " choice

case $choice in
    1)
        echo ""
        echo "启动数据采集..."
        echo "提示：请确保棋盘格已固定在机械臂末端"
        sleep 2
        python hand_eye_calibration.py
        ;;
    2)
        echo ""
        echo "启动标定求解..."
        if [ ! -f "outputs/hand_eye_data.json" ]; then
            echo "❌ 错误：找不到数据文件 outputs/hand_eye_data.json"
            echo "   请先运行选项1进行数据采集"
            exit 1
        fi
        python hand_eye_solver.py
        ;;
    3)
        echo ""
        echo "启动标定验证..."
        if [ ! -f "outputs/camera_to_base_calibration.json" ]; then
            echo "❌ 错误：找不到标定结果文件"
            echo "   请先运行选项2进行标定求解"
            exit 1
        fi
        python test_calibration.py
        ;;
    4)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选项"
        exit 1
        ;;
esac

echo ""
echo "======================================"
echo "  操作完成"
echo "======================================"
