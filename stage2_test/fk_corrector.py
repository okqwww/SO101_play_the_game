#!/usr/bin/env python3
"""
FK 位置校正模块
===============

基于 check_urdf_error.py 的实测数据，拟合一个 2D 仿射变换，
校正 FK 输出位置与物理真实位置之间的系统性偏差。

模型:  Real_XY = A @ FK_XY + b       (正向：FK → 真实)
逆向:  FK_XY   = A_inv @ (Real_XY - b) (逆向：真实 → FK，用于 IK 目标修正)

使用流程:
  1. 运行 check_urdf_error.py 采集标定数据 → outputs/check_urdf_error_result.json
  2. 运行 calibrate_fk.py 拟合校正参数     → outputs/fk_correction.json
  3. 其他脚本 import FKCorrector 使用校正
"""

import json
import numpy as np
from pathlib import Path

DEFAULT_CORRECTION_FILE = "outputs/fk_correction.json"


class FKCorrector:
    """FK 位置 2D 仿射校正器"""

    def __init__(self, A: np.ndarray, b: np.ndarray):
        """
        Args:
            A: 2x2 仿射矩阵 (FK_XY → Real_XY)
            b: 2x1 平移向量 (FK_XY → Real_XY)
        """
        self.A = np.array(A, dtype=float)
        self.b = np.array(b, dtype=float).ravel()
        self.A_inv = np.linalg.inv(self.A)

    # ─── 正向校正：FK → 真实 ──────────────────────────────────

    def correct_fk(self, fk_xyz: np.ndarray) -> np.ndarray:
        """
        将 FK 输出的 XYZ 位置校正为更接近真实的位置。
        （仅校正 XY，Z 保持不变）

        Args:
            fk_xyz: FK 位置 [x, y, z]，单位米

        Returns:
            校正后的 [x, y, z]，单位米
        """
        corrected = np.array(fk_xyz, dtype=float).copy()
        corrected[:2] = self.A @ fk_xyz[:2] + self.b
        return corrected

    # ─── 逆向校正：真实 → FK（用于 IK 目标修正）──────────────

    def target_for_ik(self, real_xyz: np.ndarray) -> np.ndarray:
        """
        给定期望的 *真实* 目标位置，计算 IK 应该求解的 *FK空间* 目标。
        这样 IK 解出的关节角度驱动机械臂后，末端会到达期望的真实位置。

        Args:
            real_xyz: 期望的真实目标位置 [x, y, z]，单位米

        Returns:
            FK 空间的目标 [x, y, z]，单位米（喂给 IK 求解器）
        """
        target = np.array(real_xyz, dtype=float).copy()
        target[:2] = self.A_inv @ (real_xyz[:2] - self.b)
        return target

    # ─── 序列化 ────────────────────────────────────────────────

    def save(self, path: str = DEFAULT_CORRECTION_FILE):
        """保存校正参数到 JSON"""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        data = {
            "description": "FK 2D affine correction: Real_XY = A @ FK_XY + b",
            "A": self.A.tolist(),
            "b": self.b.tolist(),
            "A_inv": self.A_inv.tolist(),
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"✅ FK 校正参数已保存: {path}")

    @classmethod
    def load(cls, path: str = DEFAULT_CORRECTION_FILE) -> "FKCorrector":
        """从 JSON 加载校正参数"""
        with open(path) as f:
            data = json.load(f)
        return cls(A=np.array(data["A"]), b=np.array(data["b"]))

    # ─── 拟合 ──────────────────────────────────────────────────

    @classmethod
    def fit(cls, fk_points: np.ndarray, real_points: np.ndarray) -> "FKCorrector":
        """
        从 FK 位置和真实位置的配对数据拟合仿射校正。

        模型: Real_XY = A @ FK_XY + b (最小二乘拟合)

        Args:
            fk_points:   Nx2 数组，FK 位置的 [x, y]（单位米）
            real_points: Nx2 数组，真实位置的 [x, y]（单位米）

        Returns:
            拟合好的 FKCorrector 实例
        """
        n = len(fk_points)
        assert n >= 3, f"至少需要 3 个点，当前 {n} 个"
        assert fk_points.shape == (n, 2)
        assert real_points.shape == (n, 2)

        # 构造设计矩阵: [FK_x, FK_y, 1]
        M = np.column_stack([fk_points, np.ones(n)])  # Nx3

        # 分别拟合 Real_X 和 Real_Y
        #   Real_X = a11*FK_x + a12*FK_y + b1
        #   Real_Y = a21*FK_x + a22*FK_y + b2
        params_x, residuals_x, _, _ = np.linalg.lstsq(M, real_points[:, 0], rcond=None)
        params_y, residuals_y, _, _ = np.linalg.lstsq(M, real_points[:, 1], rcond=None)

        A = np.array([
            [params_x[0], params_x[1]],
            [params_y[0], params_y[1]],
        ])
        b = np.array([params_x[2], params_y[2]])

        return cls(A=A, b=b)

    @classmethod
    def from_calibration_data(
        cls,
        calibration_file: str = "outputs/check_urdf_error_result.json",
    ) -> "FKCorrector":
        """
        直接从 check_urdf_error.py 的结果 JSON 拟合校正。

        Args:
            calibration_file: check_urdf_error_result.json 的路径

        Returns:
            拟合好的 FKCorrector 实例
        """
        with open(calibration_file) as f:
            data = json.load(f)

        fk_list = []
        real_list = []
        for r in data["records"]:
            fk_xy = r["fk_mm"][:2]      # [fk_x, fk_y] mm
            real_xy = r["real_mm"][:2]   # [real_x, real_y] mm
            # 跳过缺失数据
            if any(v is None or (isinstance(v, float) and np.isnan(v)) for v in real_xy):
                continue
            fk_list.append(fk_xy)
            real_list.append(real_xy)

        fk_pts = np.array(fk_list) / 1000.0    # 转换为米
        real_pts = np.array(real_list) / 1000.0

        print(f"📊 加载了 {len(fk_pts)} 组标定数据点")
        corrector = cls.fit(fk_pts, real_pts)
        return corrector

    # ─── 诊断 ──────────────────────────────────────────────────

    def print_diagnostics(self, fk_points: np.ndarray, real_points: np.ndarray):
        """打印校正效果诊断信息"""
        n = len(fk_points)
        print("\n" + "=" * 65)
        print("  FK 校正诊断")
        print("=" * 65)

        print(f"\n  仿射矩阵 A (FK→Real):")
        print(f"    [{self.A[0, 0]:+.6f}  {self.A[0, 1]:+.6f}]")
        print(f"    [{self.A[1, 0]:+.6f}  {self.A[1, 1]:+.6f}]")
        print(f"  平移 b: [{self.b[0]*1000:+.2f}, {self.b[1]*1000:+.2f}] mm")

        # 计算校正前后的误差
        print(f"\n  {'点':>4}  {'原始 err_X':>10}  {'原始 err_Y':>10}  {'校正后 err_X':>12}  {'校正后 err_Y':>12}")
        print("  " + "-" * 60)

        raw_errs = []
        corrected_errs = []
        for i in range(n):
            fk_xy = fk_points[i]
            real_xy = real_points[i]

            raw_err = (fk_xy - real_xy) * 1000  # mm
            corrected_xy = self.A @ fk_xy + self.b
            corr_err = (corrected_xy - real_xy) * 1000  # mm

            raw_errs.append(raw_err)
            corrected_errs.append(corr_err)

            print(f"  #{i+1:>3}  {raw_err[0]:>+10.2f}  {raw_err[1]:>+10.2f}"
                  f"  {corr_err[0]:>+12.2f}  {corr_err[1]:>+12.2f}")

        raw_errs = np.array(raw_errs)
        corrected_errs = np.array(corrected_errs)

        print(f"\n  校正前 - X 均值: {np.mean(raw_errs[:, 0]):+.2f} mm, "
              f"std: {np.std(raw_errs[:, 0]):.2f} mm")
        print(f"  校正前 - Y 均值: {np.mean(raw_errs[:, 1]):+.2f} mm, "
              f"std: {np.std(raw_errs[:, 1]):.2f} mm")
        print(f"  校正前 - 总 RMSE: {np.sqrt(np.mean(raw_errs**2)):.2f} mm")

        print(f"\n  校正后 - X 均值: {np.mean(corrected_errs[:, 0]):+.2f} mm, "
              f"std: {np.std(corrected_errs[:, 0]):.2f} mm")
        print(f"  校正后 - Y 均值: {np.mean(corrected_errs[:, 1]):+.2f} mm, "
              f"std: {np.std(corrected_errs[:, 1]):.2f} mm")
        print(f"  校正后 - 总 RMSE: {np.sqrt(np.mean(corrected_errs**2)):.2f} mm")

        improvement = (1 - np.sqrt(np.mean(corrected_errs**2)) /
                       np.sqrt(np.mean(raw_errs**2))) * 100
        print(f"\n  📈 RMSE 改善: {improvement:.1f}%")
