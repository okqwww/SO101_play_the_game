#!/usr/bin/env python3
"""
打地鼠检测器
============

检测方法：绿色背景消失法
  - 地鼠弹出时，会遮挡洞口上方的绿色背景
  - 检测每个洞口上方区域的绿色像素比例
  - 绿色比例低 → 有地鼠

使用流程（从项目根目录运行）:
  1. 标定模式（每次游戏前运行一次）:
     python stage2_test/mole_detector.py --calibrate

  2. 仅调参模式（加载已有标定，微调HSV参数）:
     python stage2_test/mole_detector.py --tune

  3. 检测测试模式（验证检测效果）:
     python stage2_test/mole_detector.py --test

  4. 在其他脚本中调用:
     from mole_detector import MoleDetector
     detector = MoleDetector.from_calibration("outputs/mole_calibration.json")
     moles, ratios = detector.detect(frame_bgr)
"""

import numpy as np
import cv2
import json
import time
from pathlib import Path


class MoleDetector:
    """打地鼠检测器：基于绿色背景消失法"""

    def __init__(
        self,
        hole_centers: list,
        monitor_width: int = 60,
        monitor_height: int = 80,
        monitor_offset_y: int = 0,
        green_hsv_low: tuple = (35, 40, 40),
        green_hsv_high: tuple = (85, 255, 255),
        green_ratio_threshold: float = 0.3,
    ):
        """
        Args:
            hole_centers: 9个洞的中心像素坐标 [(x1,y1), ..., (x9,y9)]
            monitor_width: 监控区域宽度（像素）
            monitor_height: 监控区域高度（像素）
            monitor_offset_y: 监控区域底边相对洞中心的Y偏移（负=上移，正=下移）
            green_hsv_low: 绿色HSV下界 (H, S, V)
            green_hsv_high: 绿色HSV上界 (H, S, V)
            green_ratio_threshold: 绿色比例阈值，低于此值判定有地鼠
        """
        self.hole_centers = [tuple(c) for c in hole_centers]
        self.monitor_width = monitor_width
        self.monitor_height = monitor_height
        self.monitor_offset_y = monitor_offset_y
        self.green_hsv_low = np.array(green_hsv_low, dtype=np.uint8)
        self.green_hsv_high = np.array(green_hsv_high, dtype=np.uint8)
        self.green_ratio_threshold = green_ratio_threshold

        self.monitor_regions = self._compute_monitor_regions()

    def _compute_monitor_regions(self) -> list:
        """
        计算每个洞的监控区域。

        监控区域在洞中心的上方（地鼠从洞口向上探出）。
        区域底边在 cy + offset_y，区域顶边在 cy + offset_y - height。

        Returns:
            list of (x1, y1, x2, y2) 矩形区域
        """
        regions = []
        w, h = self.monitor_width, self.monitor_height
        for cx, cy in self.hole_centers:
            x1 = int(cx - w // 2)
            y1 = int(cy + self.monitor_offset_y - h)
            x2 = int(cx + w // 2)
            y2 = int(cy + self.monitor_offset_y)
            regions.append((x1, y1, x2, y2))
        return regions

    def auto_compute_monitor_size(self):
        """
        根据9个洞的间距自动计算监控区域大小。

        不依赖行列假设，自动找出3x3网格的两个主轴间距。
        适用于手机横放、竖放、或任意旋转的情况。
        """
        if len(self.hole_centers) != 9:
            print("⚠️  需要9个洞才能自动计算")
            return

        centers = np.array(self.hole_centers, dtype=float)

        # 计算所有洞两两之间的距离
        from scipy.spatial.distance import cdist
        dists = cdist(centers, centers)

        # 找最近邻距离（排除自身=0），取每个点最近邻距离的中位数
        np.fill_diagonal(dists, np.inf)
        nearest_dists = dists.min(axis=1)
        min_spacing = np.median(nearest_dists)

        # 3x3网格有两种间距: 行间距和列间距
        # 找所有"接近最小间距"的距离对（±50%），取平均作为小间距
        all_dists = dists[np.triu_indices(9, k=1)]
        close_mask = all_dists < min_spacing * 1.5
        small_spacing = np.mean(all_dists[close_mask]) if np.any(close_mask) else min_spacing

        # 监控区域大小 = 最小间距的一定比例
        self.monitor_width = max(10, int(small_spacing * 0.5))
        self.monitor_height = max(10, int(small_spacing * 0.6))
        self.monitor_offset_y = 0  # 底边在洞中心

        # 重新计算区域
        self.monitor_regions = self._compute_monitor_regions()

        print(f"✅ 自动计算监控区域大小:")
        print(f"   洞间最小间距: {small_spacing:.0f} px")
        print(f"   监控区域: {self.monitor_width} x {self.monitor_height} px")

    def detect(self, frame_bgr: np.ndarray) -> tuple:
        """
        检测哪些洞有地鼠。

        Args:
            frame_bgr: BGR格式的图像帧

        Returns:
            mole_indices: 有地鼠的洞的索引列表 (0-8)
            green_ratios: 每个洞的绿色比例列表 (用于调试)
        """
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h_img, w_img = frame_bgr.shape[:2]

        mole_indices = []
        green_ratios = []

        for i, (x1, y1, x2, y2) in enumerate(self.monitor_regions):
            # 裁剪到图像范围内
            x1c = max(0, x1)
            y1c = max(0, y1)
            x2c = min(w_img, x2)
            y2c = min(h_img, y2)

            if x2c <= x1c or y2c <= y1c:
                green_ratios.append(1.0)  # 区域无效，当作没有地鼠
                continue

            roi = hsv[y1c:y2c, x1c:x2c]

            # 绿色掩码
            mask = cv2.inRange(roi, self.green_hsv_low, self.green_hsv_high)
            green_ratio = np.sum(mask > 0) / mask.size if mask.size > 0 else 1.0
            green_ratios.append(green_ratio)

            if green_ratio < self.green_ratio_threshold:
                mole_indices.append(i)

        return mole_indices, green_ratios

    def draw_debug(
        self,
        frame_bgr: np.ndarray,
        mole_indices: list = None,
        green_ratios: list = None,
    ) -> np.ndarray:
        """
        在帧上绘制调试信息（监控区域、绿色比例、检测结果）。

        Args:
            frame_bgr: BGR格式的图像帧
            mole_indices: detect()返回的地鼠索引
            green_ratios: detect()返回的绿色比例

        Returns:
            标注后的BGR图像
        """
        if mole_indices is None or green_ratios is None:
            mole_indices, green_ratios = self.detect(frame_bgr)

        debug_frame = frame_bgr.copy()

        for i, ((cx, cy), (x1, y1, x2, y2)) in enumerate(
            zip(self.hole_centers, self.monitor_regions)
        ):
            is_mole = i in mole_indices
            color = (0, 0, 255) if is_mole else (0, 255, 0)  # 红=地鼠, 绿=空洞

            # 画监控区域矩形
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)

            # 画洞中心标记
            cv2.circle(debug_frame, (int(cx), int(cy)), 5, (255, 255, 0), -1)

            # 显示编号和绿色比例
            ratio = green_ratios[i] if i < len(green_ratios) else 0
            label = f"#{i+1} G:{ratio:.0%}"
            if is_mole:
                label += " MOLE!"
            cv2.putText(
                debug_frame, label, (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
            )

        # 底部汇总
        mole_str = (
            f"Moles at holes: {[i+1 for i in mole_indices]}"
            if mole_indices
            else "No moles detected"
        )
        cv2.putText(
            debug_frame, mole_str, (10, debug_frame.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
        )

        return debug_frame

    # ── 序列化 ──────────────────────────────────────────────

    def save_calibration(self, filepath: str):
        """保存标定参数到JSON文件。"""
        data = {
            "hole_centers": self.hole_centers,
            "monitor_width": self.monitor_width,
            "monitor_height": self.monitor_height,
            "monitor_offset_y": self.monitor_offset_y,
            "green_hsv_low": self.green_hsv_low.tolist(),
            "green_hsv_high": self.green_hsv_high.tolist(),
            "green_ratio_threshold": self.green_ratio_threshold,
        }
        Path(filepath).parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)
        print(f"✅ 标定参数已保存: {filepath}")

    @classmethod
    def from_calibration(cls, filepath: str) -> "MoleDetector":
        """从JSON文件加载标定参数，创建检测器实例。"""
        with open(filepath) as f:
            data = json.load(f)
        return cls(
            hole_centers=data["hole_centers"],
            monitor_width=data["monitor_width"],
            monitor_height=data["monitor_height"],
            monitor_offset_y=data["monitor_offset_y"],
            green_hsv_low=tuple(data["green_hsv_low"]),
            green_hsv_high=tuple(data["green_hsv_high"]),
            green_ratio_threshold=data["green_ratio_threshold"],
        )


# ═══════════════════════════════════════════════════════════
#  交互式工具函数
# ═══════════════════════════════════════════════════════════


def calibrate_holes(camera) -> list | None:
    """
    交互式标定9个洞的位置。

    显示相机画面，让用户依次点击9个洞的中心。
    支持 'r' 撤销上一步，'q' 退出。

    Args:
        camera: 已连接的相机实例（read() 返回 RGB 格式）

    Returns:
        9个洞的中心坐标 [(x1,y1), ..., (x9,y9)]，或 None（用户取消）
    """
    hole_names = [
        "左上(1)", "中上(2)", "右上(3)",
        "左中(4)", "正中(5)", "右中(6)",
        "左下(7)", "中下(8)", "右下(9)",
    ]

    # 先刷掉缓冲帧，再拍一帧用于标定
    for _ in range(10):
        camera.read()
        time.sleep(0.05)
    frame_rgb = camera.read()
    if isinstance(frame_rgb, dict):
        frame_rgb = frame_rgb["frame"]
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    hole_centers: list[tuple[int, int]] = []
    click_pos: list[tuple[int, int] | None] = [None]

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_pos[0] = (x, y)

    window_name = "Mole Hole Calibration"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 540)
    cv2.setMouseCallback(window_name, mouse_callback)

    print("\n" + "=" * 60)
    print("打地鼠洞口标定")
    print("=" * 60)
    print("请依次点击9个洞的中心位置（从左到右，从上到下）")
    print("  鼠标左键 = 标定当前洞")
    print("  'r' = 撤销上一个点")
    print("  'q' = 退出标定\n")

    i = 0
    while i < 9:
        # 绘制已标定的点
        display = frame_bgr.copy()
        for j, (cx, cy) in enumerate(hole_centers):
            cv2.circle(display, (cx, cy), 8, (0, 255, 0), -1)
            cv2.putText(
                display, str(j + 1), (cx - 5, cy + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
            )

        # 提示信息
        prompt = f"请点击洞 {hole_names[i]} 的中心"
        cv2.putText(
            display, prompt, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
        )
        cv2.putText(
            display,
            f"已标定 {i}/9 | 'r'=撤销 'q'=退出",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
        )

        cv2.imshow(window_name, display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            print("标定已取消")
            cv2.destroyWindow(window_name)
            return None

        if key == ord("r") and i > 0:
            hole_centers.pop()
            i -= 1
            click_pos[0] = None
            print(f"  ↩ 撤销: 洞 {hole_names[i]}")
            continue

        if click_pos[0] is not None:
            cx, cy = click_pos[0]
            hole_centers.append((cx, cy))
            print(f"  ✅ 洞 {hole_names[i]}: ({cx}, {cy})")
            click_pos[0] = None
            i += 1

    cv2.destroyWindow(window_name)
    print(f"\n✅ 9个洞全部标定完成!")
    return hole_centers


def tune_hsv_params(camera, detector: MoleDetector):
    """
    HSV参数调试工具。

    实时显示相机画面 + 绿色掩码 + 检测结果，提供滑块调参。

    按 's' 保存参数，按 'q' 退出。

    Args:
        camera: 已连接的相机实例
        detector: MoleDetector实例
    """
    window_name = "HSV Tuning"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 400)

    # 创建HSV和监控区域滑块
    cv2.createTrackbar("H_Low", window_name, int(detector.green_hsv_low[0]), 180, lambda x: None)
    cv2.createTrackbar("H_High", window_name, int(detector.green_hsv_high[0]), 180, lambda x: None)
    cv2.createTrackbar("S_Low", window_name, int(detector.green_hsv_low[1]), 255, lambda x: None)
    cv2.createTrackbar("S_High", window_name, int(detector.green_hsv_high[1]), 255, lambda x: None)
    cv2.createTrackbar("V_Low", window_name, int(detector.green_hsv_low[2]), 255, lambda x: None)
    cv2.createTrackbar("V_High", window_name, int(detector.green_hsv_high[2]), 255, lambda x: None)
    cv2.createTrackbar("Threshold%", window_name, int(detector.green_ratio_threshold * 100), 100, lambda x: None)
    cv2.createTrackbar("Mon_W", window_name, detector.monitor_width, 200, lambda x: None)
    cv2.createTrackbar("Mon_H", window_name, detector.monitor_height, 200, lambda x: None)
    # offset 范围 -100~+100，滑块映射为 0~200
    cv2.createTrackbar("Mon_OffY", window_name, detector.monitor_offset_y + 100, 200, lambda x: None)

    print("\n" + "=" * 60)
    print("HSV参数调试")
    print("=" * 60)
    print("左边: 原图+检测框   右边: 绿色掩码")
    print("调整滑块直到:")
    print("  - 绿色背景区域在右侧显示为白色")
    print("  - 地鼠/土堆区域在右侧显示为黑色")
    print("  - 空洞显示绿色框(G:xx%)，有地鼠显示红色框(MOLE!)")
    print("按 's' = 保存参数   'q' = 退出\n")

    while True:
        # 读取相机
        frame_rgb = camera.read()
        if isinstance(frame_rgb, dict):
            frame_rgb = frame_rgb["frame"]
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # 读取滑块值
        h_low = cv2.getTrackbarPos("H_Low", window_name)
        h_high = cv2.getTrackbarPos("H_High", window_name)
        s_low = cv2.getTrackbarPos("S_Low", window_name)
        s_high = cv2.getTrackbarPos("S_High", window_name)
        v_low = cv2.getTrackbarPos("V_Low", window_name)
        v_high = cv2.getTrackbarPos("V_High", window_name)
        threshold = cv2.getTrackbarPos("Threshold%", window_name) / 100.0
        mon_w = cv2.getTrackbarPos("Mon_W", window_name)
        mon_h = cv2.getTrackbarPos("Mon_H", window_name)
        mon_offy = cv2.getTrackbarPos("Mon_OffY", window_name) - 100

        # 更新检测器参数
        detector.green_hsv_low = np.array([h_low, s_low, v_low], dtype=np.uint8)
        detector.green_hsv_high = np.array([h_high, s_high, v_high], dtype=np.uint8)
        detector.green_ratio_threshold = threshold

        need_recompute = (
            mon_w != detector.monitor_width
            or mon_h != detector.monitor_height
            or mon_offy != detector.monitor_offset_y
        )
        if need_recompute:
            detector.monitor_width = max(10, mon_w)
            detector.monitor_height = max(10, mon_h)
            detector.monitor_offset_y = mon_offy
            detector.monitor_regions = detector._compute_monitor_regions()

        # 检测
        mole_indices, green_ratios = detector.detect(frame_bgr)

        # 左半边: 原图 + 检测框
        debug_frame = detector.draw_debug(frame_bgr, mole_indices, green_ratios)

        # 右半边: 绿色掩码
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, detector.green_hsv_low, detector.green_hsv_high)
        mask_colored = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)

        # 在掩码图上也画监控区域
        for i, (x1, y1, x2, y2) in enumerate(detector.monitor_regions):
            is_mole = i in mole_indices
            color = (0, 0, 255) if is_mole else (0, 255, 0)
            cv2.rectangle(mask_colored, (x1, y1), (x2, y2), color, 2)

        # 缩小并左右拼接
        h_img, w_img = debug_frame.shape[:2]
        scale = 0.5
        debug_small = cv2.resize(debug_frame, (int(w_img * scale), int(h_img * scale)))
        mask_small = cv2.resize(mask_colored, (int(w_img * scale), int(h_img * scale)))
        combined = np.hstack([debug_small, mask_small])

        cv2.imshow(window_name, combined)
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("s"):
            detector.save_calibration("outputs/mole_calibration.json")

    cv2.destroyWindow(window_name)


# ═══════════════════════════════════════════════════════════
#  命令行入口
# ═══════════════════════════════════════════════════════════


def _init_camera():
    """初始化OpenCV相机，返回已连接的相机实例。"""
    from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
    from lerobot.cameras.configs import ColorMode

    with open("outputs/cheap_camera_intrinsics.json") as f:
        intrinsics = json.load(f)

    camera_config = OpenCVCameraConfig(
        index_or_path=0,
        fps=10,
        width=intrinsics["image_width"],
        height=intrinsics["image_height"],
        color_mode=ColorMode.RGB,
    )
    camera = OpenCVCamera(camera_config)
    camera.connect()
    print("✅ 相机已连接")
    return camera


def main():
    """命令行入口：标定 / 调参 / 测试"""
    import argparse

    parser = argparse.ArgumentParser(description="打地鼠检测器")
    parser.add_argument("--calibrate", action="store_true", help="标定9个洞的位置 + HSV调参")
    parser.add_argument("--tune", action="store_true", help="仅HSV调参（使用已有标定）")
    parser.add_argument("--test", action="store_true", help="实时检测测试")
    parser.add_argument(
        "--config",
        default="outputs/mole_calibration.json",
        help="标定配置文件路径 (默认: outputs/mole_calibration.json)",
    )
    args = parser.parse_args()

    if not (args.calibrate or args.tune or args.test):
        parser.print_help()
        return

    camera = _init_camera()

    try:
        if args.calibrate:
            # ── 标定模式 ──────────────────────────────
            hole_centers = calibrate_holes(camera)
            if hole_centers is None:
                return

            # 创建检测器，自动计算监控区域大小
            detector = MoleDetector(hole_centers)
            detector.auto_compute_monitor_size()

            # 进入HSV调参
            print("\n进入HSV参数调试模式... (按 's' 保存, 'q' 退出)")
            tune_hsv_params(camera, detector)

        elif args.tune:
            # ── 仅调参模式 ────────────────────────────
            detector = MoleDetector.from_calibration(args.config)
            print(f"✅ 已加载标定: {args.config}")
            tune_hsv_params(camera, detector)

        elif args.test:
            # ── 实时检测测试 ──────────────────────────
            detector = MoleDetector.from_calibration(args.config)
            print(f"✅ 已加载标定: {args.config}")
            print("\n实时检测中... 按 'q' 退出\n")

            cv2.namedWindow("Mole Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Mole Detection", 960, 540)

            while True:
                frame_rgb = camera.read()
                if isinstance(frame_rgb, dict):
                    frame_rgb = frame_rgb["frame"]
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

                mole_indices, green_ratios = detector.detect(frame_bgr)
                debug = detector.draw_debug(frame_bgr, mole_indices, green_ratios)

                if mole_indices:
                    holes_str = ", ".join(str(i + 1) for i in mole_indices)
                    print(f"🔴 地鼠出现在洞: {holes_str}")

                cv2.imshow("Mole Detection", debug)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break

            cv2.destroyAllWindows()

    finally:
        camera.disconnect()
        print("✅ 相机已断开")


if __name__ == "__main__":
    main()
