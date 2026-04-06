#!/usr/bin/env python3
"""
FK 校正参数拟合
===============

从 check_urdf_error.py 的测试结果中拟合 2D 仿射校正参数，
并保存到 outputs/fk_correction.json。

使用方法（从项目根目录运行）:
  python stage2_test/error_analysis/calibrate_fk.py

前提: 已运行 check_urdf_error.py 并保存了 outputs/check_urdf_error_result.json
"""

import sys
import numpy as np
from pathlib import Path

# 将 stage2_test 加入路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from fk_corrector import FKCorrector

CALIBRATION_DATA = "outputs/check_urdf_error_result.json"
CORRECTION_OUTPUT = "outputs/fk_correction.json"


def main():
    print("\n" + "=" * 65)
    print("  FK 校正参数拟合")
    print("=" * 65)

    # ── 1. 加载并拟合 ───────────────────────────────────────
    try:
        corrector = FKCorrector.from_calibration_data(CALIBRATION_DATA)
    except FileNotFoundError:
        print(f"\n❌ 未找到标定数据: {CALIBRATION_DATA}")
        print("请先运行: python stage2_test/error_analysis/check_urdf_error.py")
        return

    # ── 2. 加载原始数据用于诊断 ─────────────────────────────
    import json
    with open(CALIBRATION_DATA) as f:
        data = json.load(f)

    fk_pts = []
    real_pts = []
    for r in data["records"]:
        fk_xy = r["fk_mm"][:2]
        real_xy = r["real_mm"][:2]
        if any(v is None or (isinstance(v, float) and np.isnan(v)) for v in real_xy):
            continue
        fk_pts.append(fk_xy)
        real_pts.append(real_xy)

    fk_pts = np.array(fk_pts) / 1000.0
    real_pts = np.array(real_pts) / 1000.0

    # ── 3. 打印诊断 ────────────────────────────────────────
    corrector.print_diagnostics(fk_pts, real_pts)

    # ── 4. 保存 ────────────────────────────────────────────
    corrector.save(CORRECTION_OUTPUT)

    # ── 5. 验证逆变换 ──────────────────────────────────────
    print("\n" + "─" * 65)
    print("  逆变换验证（Real → FK 目标）")
    print("─" * 65)
    print(f"\n  {'点':>4}  {'Real_X':>8}  {'Real_Y':>8}  →  {'IK目标_X':>10}  {'IK目标_Y':>10}")
    for i in range(len(real_pts)):
        real_xyz = np.array([real_pts[i, 0], real_pts[i, 1], 0.0])
        ik_target = corrector.target_for_ik(real_xyz)
        print(f"  #{i+1:>3}  {real_xyz[0]*1000:>+8.1f}  {real_xyz[1]*1000:>+8.1f}"
              f"  →  {ik_target[0]*1000:>+10.1f}  {ik_target[1]*1000:>+10.1f}")

    print(f"\n✅ 拟合完成！后续脚本可通过 FKCorrector.load('{CORRECTION_OUTPUT}') 使用校正。")


if __name__ == "__main__":
    main()
