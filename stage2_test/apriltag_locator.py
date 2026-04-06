#!/usr/bin/env python3
"""
AprilTag 底板自动定位模块
=========================

功能：
  利用底板四周的 AprilTag 标记，自动计算：
    1. 相机到底板表面的实时深度
    2. 9 个地鼠洞的实时像素坐标

原理：
  - Setup 阶段（运行一次）：
      a. 检测 Tags，将底板坐标系定义为"当前相机坐标系"
      b. 记录每个 Tag 的 4 个角点在底板坐标系中的 3D 坐标
      c. 用户点击 9 个地鼠洞，反投影到底板平面，记录 3D 坐标
      d. 保存到 outputs/board_config.json

  - 运行阶段（每次游戏）：
      a. 检测 Tags，用 PnP 求解 T_board_to_cam（底板→相机的变换）
      b. 将洞的底板坐标投影到像素
      c. 深度 = T_board_to_cam[2, 3]

优势：
  - 底板/手机移动后，下一帧自动重新定位，无需任何手动操作
  - 注意：相机若相对机械臂基座移动，仍需重新做手眼标定

使用方法（从项目根目录运行）：
  python stage2_test/apriltag_locator.py --setup   # 首次运行
  python stage2_test/apriltag_locator.py --test    # 验证检测效果
"""

import sys
import json
import numpy as np
import cv2
from pathlib import Path
from typing import Optional, List, Tuple, Dict

# ─────────── AprilTag 检测库 ───────────
try:
    import pupil_apriltags as apriltag_lib
    _APRILTAG_LIB = "pupil_apriltags"
except ImportError:
    try:
        import apriltag as apriltag_lib
        _APRILTAG_LIB = "apriltag"
    except ImportError:
        apriltag_lib = None
        _APRILTAG_LIB = None

TAG_FAMILY   = "tag36h11"   # 与实物一致
TAG_SIZE_M   = 0.040        # 40 mm = 0.040 m（测量的是黑色外框边长）
DEFAULT_CONFIG = "outputs/board_config.json"
DEFAULT_INTRINSICS = "outputs/cheap_camera_intrinsics.json"


class AprilTagLocator:
    """
    基于 AprilTag 的底板自动定位器。

    底板坐标系定义：
      - Setup 时相机坐标系即为底板坐标系（由第一次检测建立）
      - 所有洞坐标和 Tag 角点坐标均以此系为准
    """

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        config_file: str = DEFAULT_CONFIG,
    ):
        """
        Args:
            camera_matrix: 3×3 相机内参矩阵
            dist_coeffs:   畸变系数
            config_file:   底板配置文件路径（setup 时生成）
        """
        if _APRILTAG_LIB is None:
            raise ImportError(
                "请安装 AprilTag 库：\n  pip install pupil-apriltags"
            )

        self.K = np.array(camera_matrix, dtype=np.float64)
        self.D = np.array(dist_coeffs, dtype=np.float64)
        self.config_file = config_file
        self.board_data: Optional[Dict] = None

        # 初始化检测器
        if _APRILTAG_LIB == "pupil_apriltags":
            self.detector = apriltag_lib.Detector(
                families=TAG_FAMILY,
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
            )
        else:  # apriltag
            options = apriltag_lib.DetectorOptions(families=TAG_FAMILY)
            self.detector = apriltag_lib.Detector(options)

        # 若已有配置则加载
        if Path(config_file).exists():
            self._load_config()

    # ─────────────────────────────────────────────────────────
    #  类方法：从文件加载内参
    # ─────────────────────────────────────────────────────────

    @classmethod
    def from_intrinsics_file(
        cls,
        intrinsics_file: str = DEFAULT_INTRINSICS,
        config_file: str = DEFAULT_CONFIG,
    ) -> "AprilTagLocator":
        """从标定文件加载内参并构造 AprilTagLocator。"""
        with open(intrinsics_file) as f:
            d = json.load(f)
        K = np.array(d["camera_matrix"], dtype=np.float64)
        D = np.array(d["dist_coeffs"],   dtype=np.float64)
        return cls(K, D, config_file)

    # ─────────────────────────────────────────────────────────
    #  内部工具
    # ─────────────────────────────────────────────────────────

    def _tag_corners_in_tag_frame(self) -> np.ndarray:
        """
        返回 Tag 坐标系中 4 个角点（顺序：左下、右下、右上、左上），shape (4, 3)。
        Tag 坐标系：原点在 Tag 中心，X 向右，Y 向上，Z 朝向相机。
        """
        h = TAG_SIZE_M / 2.0
        return np.array([
            [-h, -h, 0.0],
            [ h, -h, 0.0],
            [ h,  h, 0.0],
            [-h,  h, 0.0],
        ], dtype=np.float64)

    def _detect(self, frame_bgr: np.ndarray) -> list:
        """检测图像中的 AprilTag，返回检测结果列表。"""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        fx = self.K[0, 0]; fy = self.K[1, 1]
        cx = self.K[0, 2]; cy = self.K[1, 2]

        if _APRILTAG_LIB == "pupil_apriltags":
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(fx, fy, cx, cy),
                tag_size=TAG_SIZE_M,
            )
        else:  # apriltag
            detections = self.detector.detect(gray)
            # apriltag 库需要单独估算位姿
            K_params = (fx, fy, cx, cy)
            for det in detections:
                obj_pts = self._tag_corners_in_tag_frame()
                img_pts = det.corners.astype(np.float32)
                _, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, self.K, self.D,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                R, _ = cv2.Rodrigues(rvec)
                det.pose_R = R
                det.pose_t = tvec
        return detections

    def _get_T_tag_to_cam(self, det) -> np.ndarray:
        """从检测结果提取 4×4 变换矩阵 T_tag_to_cam。"""
        R = np.array(det.pose_R, dtype=np.float64).reshape(3, 3)
        t = np.array(det.pose_t, dtype=np.float64).ravel()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = t
        return T

    def _get_corners_2d(self, det) -> np.ndarray:
        """返回 2D 像素角点，shape (4, 2)，dtype float32。"""
        return det.corners.astype(np.float32)

    def _board_plane(self, detections: list) -> Tuple[np.ndarray, np.ndarray]:
        """
        从多个 Tag 估算底板平面。
        Returns:
            normal:  单位法向量（指向相机）
            center:  平面上一点（Tag 中心平均值）
        """
        normals = []
        centers = []
        for det in detections:
            T = self._get_T_tag_to_cam(det)
            normals.append(T[:3, 2])   # Tag Z 轴 = 指向相机的法向量
            centers.append(T[:3, 3])   # Tag 中心
        normal = np.mean(normals, axis=0)
        normal /= np.linalg.norm(normal)
        center = np.mean(centers, axis=0)
        return normal, center

    def _ray_plane_intersect(
        self,
        pixel_x: float,
        pixel_y: float,
        plane_normal: np.ndarray,
        plane_point: np.ndarray,
    ) -> np.ndarray:
        """
        将像素(u, v)反投影为射线，与底板平面求交，返回相机坐标系 3D 点。
        """
        # 相机坐标系中的射线方向（未归一化）
        ray = np.linalg.inv(self.K) @ np.array([pixel_x, pixel_y, 1.0])
        ray /= np.linalg.norm(ray)

        denom = np.dot(plane_normal, ray)
        if abs(denom) < 1e-6:
            # 近似：用平面中心深度
            t = np.dot(plane_normal, plane_point)
        else:
            t = np.dot(plane_normal, plane_point) / denom

        return t * ray

    # ─────────────────────────────────────────────────────────
    #  Setup：交互式底板设置（运行一次）
    # ─────────────────────────────────────────────────────────

    def setup_board(self, camera_idx: int = 0) -> bool:
        """
        交互式底板设置向导：
          1. 检测 Tag，建立底板坐标系
          2. 用户点击 9 个地鼠洞
          3. 保存 board_config.json

        Args:
            camera_idx: cv2.VideoCapture 设备索引（默认 0）

        Returns:
            True 表示成功，False 表示用户中途取消
        """
        print("=" * 60)
        print("AprilTag 底板设置向导")
        print("=" * 60)
        print(f"Tag 家族: {TAG_FAMILY}  | Tag 尺寸: {TAG_SIZE_M * 1000:.0f} mm")
        print("将底板放到相机下方，确保至少 2 个 Tag 可见")
        print("按 [空格] 采集图像，按 [ESC] 退出")
        print("-" * 60)

        cap = cv2.VideoCapture(camera_idx)
        if not cap.isOpened():
            print(f"❌ 无法打开相机 {camera_idx}")
            return False

        # ── 阶段 1：采集帧并确认 Tag 检测 ──
        detections = []
        frame_bgr  = None
        win_name   = "AprilTag Setup"

        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 读取相机失败")
                cap.release()
                return False
            frame_bgr = frame.copy()
            dets = self._detect(frame)

            vis = frame.copy()
            for d in dets:
                corners = self._get_corners_2d(d).astype(int)
                cv2.polylines(vis, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
                ctr = corners.mean(axis=0).astype(int)
                cv2.putText(
                    vis, f"ID:{d.tag_id}",
                    tuple(ctr - [20, 0]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2,
                )
            cv2.putText(
                vis,
                f"检测到 {len(dets)} 个Tag  [空格] 采集 [ESC] 退出",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2,
            )
            cv2.imshow(win_name, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                if len(dets) >= 2:
                    detections = dets
                    print(f"✅ 采集成功！检测到 {len(dets)} 个 Tags: "
                          f"{sorted([d.tag_id for d in dets])}")
                    break
                else:
                    print(f"  ⚠️  只检测到 {len(dets)} 个 Tag，请调整位置后再试")
            elif key == 27:  # ESC
                cv2.destroyAllWindows()
                cap.release()
                return False

        # ── 阶段 2：建立底板坐标系 ──
        #    底板坐标系 = 当前相机坐标系（setup 时相机帧）
        #    每个 Tag 的角点在底板坐标系中的 3D 坐标：
        #      corners_board = T_tag_to_cam @ corners_tag_frame
        corners_tag_frame = self._tag_corners_in_tag_frame()   # 4×3

        tag_data: Dict[str, Dict] = {}
        for det in detections:
            T = self._get_T_tag_to_cam(det)  # tag → cam (= board)
            # 角点在底板坐标系（= 当前相机坐标系）中的位置
            corners_board = (T[:3, :3] @ corners_tag_frame.T + T[:3, 3:]).T  # 4×3
            tag_data[str(det.tag_id)] = {
                "corners_board_m": corners_board.tolist()
            }
            print(f"   Tag {det.tag_id}: 底板坐标 中心 = "
                  f"[{T[0,3]*1000:.1f}, {T[1,3]*1000:.1f}, {T[2,3]*1000:.1f}] mm")

        # ── 阶段 3：用户点击 9 个地鼠洞 ──
        normal, center = self._board_plane(detections)
        print(f"\n底板平面法向量: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]")
        print(f"底板中心深度:   {np.dot(normal, center) * 1000:.1f} mm")
        print("\n请按顺序点击 9 个地鼠洞中心（左键点击 / 右键撤销）")
        print("建议顺序：左上→中上→右上→左中→中中→右中→左下→中下→右下")

        clicked_pixels: List[Tuple[float, float]] = []

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                clicked_pixels.append((float(x), float(y)))
                print(f"  洞 #{len(clicked_pixels)}: 像素 ({x}, {y})")
            elif event == cv2.EVENT_RBUTTONDOWN and clicked_pixels:
                removed = clicked_pixels.pop()
                print(f"  撤销洞 #{len(clicked_pixels)+1}: 像素 ({removed[0]:.0f}, {removed[1]:.0f})")

        cv2.setMouseCallback(win_name, on_mouse)

        while len(clicked_pixels) < 9:
            vis = frame_bgr.copy()
            for i, (x, y) in enumerate(clicked_pixels):
                cv2.circle(vis, (int(x), int(y)), 3, (0, 255, 0), -1)
                cv2.putText(
                    vis, str(i + 1), (int(x) + 5, int(y) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                )
            cv2.putText(
                vis,
                f"点击洞 #{len(clicked_pixels)+1}/9  [右键] 撤销  [ESC] 取消",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2,
            )
            cv2.imshow(win_name, vis)
            if cv2.waitKey(30) & 0xFF == 27:
                cv2.destroyAllWindows()
                cap.release()
                return False

        # ── 阶段 4：反投影像素到底板平面 ──
        hole_positions_board: List[List[float]] = []
        for i, (px, py) in enumerate(clicked_pixels):
            pt_board = self._ray_plane_intersect(px, py, normal, center)
            hole_positions_board.append(pt_board.tolist())
            print(f"   洞 #{i+1}: 像素({px:.0f},{py:.0f}) → "
                  f"底板坐标({pt_board[0]*1000:.1f}, {pt_board[1]*1000:.1f}, "
                  f"{pt_board[2]*1000:.1f}) mm")

        # ── 阶段 5：保存配置 ──
        config = {
            "tag_family":  TAG_FAMILY,
            "tag_size_m":  TAG_SIZE_M,
            "tags":        tag_data,
            "hole_positions_board_m": hole_positions_board,
        }
        Path(self.config_file).parent.mkdir(parents=True, exist_ok=True)
        with open(self.config_file, "w") as f:
            json.dump(config, f, indent=2)
        self.board_data = config

        print(f"\n✅ 底板配置已保存: {self.config_file}")

        # 最后展示确认画面
        vis = frame_bgr.copy()
        for i, (x, y) in enumerate(clicked_pixels):
            cv2.circle(vis, (int(x), int(y)), 3, (0, 255, 0), -1)
            cv2.putText(
                vis, str(i + 1), (int(x) + 5, int(y) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )
        cv2.putText(
            vis, "设置完成！按任意键退出",
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2,
        )
        cv2.imshow(win_name, vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cap.release()
        return True

    # ─────────────────────────────────────────────────────────
    #  加载/保存
    # ─────────────────────────────────────────────────────────

    def _load_config(self):
        with open(self.config_file) as f:
            self.board_data = json.load(f)
        n_tags  = len(self.board_data["tags"])
        n_holes = len(self.board_data["hole_positions_board_m"])
        print(f"✅ 已加载底板配置: {self.config_file}")
        print(f"   Tags: {sorted(int(k) for k in self.board_data['tags'].keys())}  "
              f"| 洞数: {n_holes}")

    # ─────────────────────────────────────────────────────────
    #  Locate：运行时定位（每帧调用）
    # ─────────────────────────────────────────────────────────

    def locate(
        self, frame_bgr: np.ndarray
    ) -> Tuple[List[Tuple[float, float]], float]:
        """
        在当前帧中检测底板，返回 9 个洞的像素坐标和深度。

        Args:
            frame_bgr: BGR 图像帧

        Returns:
            hole_pixels: [(u, v), ...] 9 个洞的像素坐标
            depth_m:     float，相机到底板表面的深度（米，= T_board_to_cam Z 分量）

        Raises:
            RuntimeError: 未找到配置文件或未检测到有效 Tag
        """
        if self.board_data is None:
            raise RuntimeError(
                "未找到底板配置，请先运行：\n"
                "  python stage2_test/apriltag_locator.py --setup"
            )

        # 检测
        detections = self._detect(frame_bgr)
        if not detections:
            raise RuntimeError("未在画面中检测到 AprilTag")

        # 筛选出 board_data 中有记录的 Tag
        known_ids = set(self.board_data["tags"].keys())
        valid_dets = [d for d in detections if str(d.tag_id) in known_ids]
        if not valid_dets:
            raise RuntimeError(
                f"检测到 Tags {[d.tag_id for d in detections]}，"
                f"但均不在配置中（配置 IDs: {sorted(int(k) for k in known_ids)}）"
            )

        # ── PnP：底板坐标系 → 当前相机坐标系 ──
        pts_3d_list = []
        pts_2d_list = []
        for det in valid_dets:
            corners_board = np.array(
                self.board_data["tags"][str(det.tag_id)]["corners_board_m"],
                dtype=np.float64,
            )   # (4, 3)
            corners_2d = self._get_corners_2d(det)   # (4, 2)
            pts_3d_list.append(corners_board)
            pts_2d_list.append(corners_2d)

        pts_3d = np.vstack(pts_3d_list)   # (N*4, 3)
        pts_2d = np.vstack(pts_2d_list)   # (N*4, 2)

        success, rvec, tvec = cv2.solvePnP(
            pts_3d, pts_2d, self.K, self.D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            raise RuntimeError("PnP 位姿求解失败")

        R, _ = cv2.Rodrigues(rvec)
        depth_m = float(tvec[2])   # 底板原点在相机坐标系的 Z 分量

        # ── 投影洞坐标到当前帧像素 ──
        holes_board = np.array(
            self.board_data["hole_positions_board_m"], dtype=np.float64
        )   # (9, 3)

        # 底板坐标 → 当前相机坐标：P_cam = R @ P_board + t
        holes_cam = (R @ holes_board.T + tvec).T   # (9, 3)

        hole_pixels: List[Tuple[float, float]] = []
        for pt in holes_cam:
            u = self.K[0, 0] * pt[0] / pt[2] + self.K[0, 2]
            v = self.K[1, 1] * pt[1] / pt[2] + self.K[1, 2]
            hole_pixels.append((float(u), float(v)))

        return hole_pixels, depth_m

    # ─────────────────────────────────────────────────────────
    #  可视化
    # ─────────────────────────────────────────────────────────

    def draw_detection(
        self,
        frame_bgr: np.ndarray,
        hole_pixels: List[Tuple[float, float]],
        depth_m: Optional[float] = None,
    ) -> np.ndarray:
        """在图像上绘制检测到的洞位置。"""
        vis = frame_bgr.copy()
        for i, (u, v) in enumerate(hole_pixels):
            cv2.circle(vis, (int(u), int(v)), 3, (0, 255, 255), -1)
            cv2.putText(
                vis, str(i + 1),
                (int(u) + 5, int(v) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
            )
        if depth_m is not None:
            cv2.putText(
                vis, f"depth = {depth_m*1000:.0f} mm",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2,
            )
        return vis

    def draw_tags(self, frame_bgr: np.ndarray) -> np.ndarray:
        """在图像上绘制检测到的 Tag 边框和 ID。"""
        detections = self._detect(frame_bgr)
        vis = frame_bgr.copy()
        for d in detections:
            corners = self._get_corners_2d(d).astype(int)
            cv2.polylines(vis, [corners.reshape(-1, 1, 2)], True, (255, 128, 0), 2)
            ctr = corners.mean(axis=0).astype(int)
            cv2.putText(
                vis, f"ID:{d.tag_id}",
                tuple(ctr - [20, 0]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 128, 0), 2,
            )
        return vis


# ─────────────────────────────────────────────────────────────
#  命令行入口
# ─────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="AprilTag 底板自动定位工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python stage2_test/apriltag_locator.py --setup    首次运行，建立底板配置
  python stage2_test/apriltag_locator.py --test     实时验证检测效果
        """,
    )
    parser.add_argument("--setup",  action="store_true", help="运行底板设置向导")
    parser.add_argument("--test",   action="store_true", help="实时验证检测")
    parser.add_argument("--cam",    type=int, default=0, help="相机设备号 (默认 0)")
    args = parser.parse_args()

    # 加载内参
    try:
        locator = AprilTagLocator.from_intrinsics_file()
    except FileNotFoundError as e:
        print(f"❌ 找不到相机内参文件: {e}")
        print("   请先运行相机标定：python stage1_test/calib/calibrate_camera.py")
        sys.exit(1)
    except ImportError as e:
        print(f"❌ {e}")
        sys.exit(1)

    if args.setup:
        ok = locator.setup_board(camera_idx=args.cam)
        sys.exit(0 if ok else 1)

    elif args.test:
        print("实时检测模式（按 [ESC] 退出）")
        cap = cv2.VideoCapture(args.cam)
        if not cap.isOpened():
            print(f"❌ 无法打开相机 {args.cam}")
            sys.exit(1)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 先画 Tag
            vis = locator.draw_tags(frame)

            try:
                pixels, depth = locator.locate(frame)
                vis = locator.draw_detection(vis, pixels, depth)
                status = f"✅ 定位成功  depth={depth*1000:.0f}mm  {len(pixels)}洞"
            except RuntimeError as e:
                cv2.putText(
                    vis, f"❌ {e}",
                    (20, frame.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                )
                status = "未定位"

            print(f"\r{status}     ", end="", flush=True)
            cv2.imshow("AprilTag Locator Test", vis)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        print()
        cap.release()
        cv2.destroyAllWindows()

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
