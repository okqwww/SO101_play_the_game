#!/bin/bash
# SO101均匀网格IK精度测试一键脚本

echo "========================================================================"
echo "SO101 均匀网格IK精度测试"
echo "========================================================================"
echo ""
echo "测试规格："
echo "  - X轴: 0.15m ~ 0.41m (14个点, 间隔0.02m)"
echo "  - Y轴: -0.2m ~ 0.2m (21个点, 间隔0.02m)"
echo "  - Z轴: 0.005m, 0.015m, 0.025m (3个高度)"
echo "  - 总点数: 882个点"
echo "  - 预计时间: 约37分钟"
echo ""
echo "========================================================================"
echo ""

# 激活conda环境
echo "正在激活lerobot环境..."
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

if [ $? -ne 0 ]; then
    echo "❌ 无法激活lerobot环境"
    exit 1
fi

echo "✅ 环境激活成功"
echo ""

# 运行测试
echo "开始运行测试..."
echo "⚠️  请确保："
echo "  1. 机械臂已连接并上电"
echo "  2. 工作空间内无障碍物"
echo "  3. 测试期间不要触碰机械臂"
echo ""

python test_ik_accuracy_uniform.py

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ 测试失败或被中断"
    exit 1
fi

echo ""
echo "========================================================================"
echo "测试完成！"
echo "========================================================================"
echo ""

# 询问是否生成可视化
read -p "是否立即生成可视化图表？(y/n): " response

if [ "$response" = "y" ] || [ "$response" = "Y" ]; then
    echo ""
    echo "正在生成可视化图表..."
    python visualize_uniform_results.py
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "✅ 可视化完成！"
        echo ""
        echo "📂 结果文件位置："
        echo "  - 测试数据: outputs/ik_accuracy_test_uniform/"
        echo "  - 可视化图表: outputs/ik_accuracy_test_uniform/visualizations/"
        echo ""
        echo "💡 建议查看："
        echo "  1. 3d_precision_map.png - 3D精度分布图（绿色=<10mm）"
        echo "  2. error_heatmap.png - 误差热力图"
        echo "  3. error_distribution_analysis.png - 误差分析"
    else
        echo "❌ 可视化失败"
    fi
else
    echo ""
    echo "跳过可视化。稍后可手动运行："
    echo "  python visualize_uniform_results.py"
fi

echo ""
echo "========================================================================"
echo "完成！"
echo "========================================================================"
