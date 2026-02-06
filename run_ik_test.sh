#!/bin/bash

# SO101 IK测试运行脚本

echo "============================================"
echo "  SO101 逆运动学（IK）测试程序"
echo "============================================"
echo ""

# 激活conda环境并运行测试
conda run -n lerobot python /home/zyj/lerobot/test_ik_movement.py "$@"
