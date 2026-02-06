#!/usr/bin/env python3
"""
Hand-Eye Calibration Result Visualization
==========================================

Features:
1. Display camera position relative to robot base
2. Show spatial distribution of calibration poses
3. Analyze calibration consistency
4. 3D visualization of coordinate relationships
"""

import numpy as np
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
from scipy.spatial.transform import Rotation


class HandEyeVisualizer:
    """Hand-Eye Calibration Visualizer"""
    
    def __init__(
        self,
        data_file: str = "outputs/hand_eye_data.json",
        result_file: str = "outputs/camera_to_base_calibration.json"
    ):
        """
        Args:
            data_file: Calibration data file
            result_file: Calibration result file
        """
        # Load calibration data
        print(f"Loading calibration data: {data_file}")
        with open(data_file, 'r') as f:
            self.calibration_data = json.load(f)
        print(f"  ✅ Loaded {len(self.calibration_data)} data points")
        
        # Load calibration result
        print(f"Loading calibration result: {result_file}")
        with open(result_file, 'r') as f:
            calib_result = json.load(f)
        
        self.T_cam_to_base = np.array(calib_result["T_cam_to_base"])
        self.cam_position = np.array([
            calib_result["translation_m"]["x"],
            calib_result["translation_m"]["y"],
            calib_result["translation_m"]["z"]
        ])
        self.cam_euler = calib_result["euler_angles_deg"]
        
        print(f"  ✅ Camera position: [{self.cam_position[0]:.3f}, {self.cam_position[1]:.3f}, {self.cam_position[2]:.3f}]m")
        print(f"  ✅ Camera orientation: Roll={self.cam_euler['roll']:.1f}°, Pitch={self.cam_euler['pitch']:.1f}°, Yaw={self.cam_euler['yaw']:.1f}°")
        
    def plot_coordinate_frame(self, ax, T, label, scale=0.05, colors=['r', 'g', 'b']):
        """Plot coordinate frame"""
        origin = T[:3, 3]
        
        # X, Y, Z axes
        for i, color in enumerate(colors):
            axis = T[:3, i] * scale
            ax.quiver(origin[0], origin[1], origin[2],
                     axis[0], axis[1], axis[2],
                     color=color, arrow_length_ratio=0.3, linewidth=2)
        
        # Label
        ax.text(origin[0], origin[1], origin[2], label,
               fontsize=10, fontweight='bold')
    
    def plot_3d_setup(self):
        """Plot 3D setup: base, camera, end-effector positions"""
        print("\nGenerating 3D setup visualization...")
        
        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. Plot base coordinate frame (origin)
        T_base = np.eye(4)
        self.plot_coordinate_frame(ax, T_base, 'Base', scale=0.1, colors=['red', 'green', 'blue'])
        
        # 2. Plot camera coordinate frame
        self.plot_coordinate_frame(ax, self.T_cam_to_base, 'Camera', scale=0.08, colors=['orange', 'cyan', 'purple'])
        
        # 3. Plot all end-effector positions from calibration poses
        end_positions = []
        board_positions_in_base = []
        
        for data in self.calibration_data:
            # End-effector position
            T_base_to_end = np.array(data["T_base_to_end"])
            end_pos = T_base_to_end[:3, 3]
            end_positions.append(end_pos)
            
            # Checkerboard position in camera -> convert to base frame
            T_cam_to_board = np.array(data["T_cam_to_board"])
            T_base_to_board = self.T_cam_to_base @ T_cam_to_board
            board_pos = T_base_to_board[:3, 3]
            board_positions_in_base.append(board_pos)
        
        end_positions = np.array(end_positions)
        board_positions_in_base = np.array(board_positions_in_base)
        
        # Plot end-effector positions
        ax.scatter(end_positions[:, 0], end_positions[:, 1], end_positions[:, 2],
                  c='blue', s=100, alpha=0.6, label='End-Effector Positions', marker='o')
        
        # Plot checkerboard positions (computed from camera observation + calibration)
        ax.scatter(board_positions_in_base[:, 0], board_positions_in_base[:, 1], board_positions_in_base[:, 2],
                  c='green', s=100, alpha=0.6, label='Checkerboard Positions (Camera View)', marker='^')
        
        # Draw connection lines (end to board, should be close)
        for i in range(len(end_positions)):
            ax.plot([end_positions[i, 0], board_positions_in_base[i, 0]],
                   [end_positions[i, 1], board_positions_in_base[i, 1]],
                   [end_positions[i, 2], board_positions_in_base[i, 2]],
                   'gray', alpha=0.3, linestyle='--')
        
        # 4. Plot camera viewing direction (Z-axis)
        cam_pos = self.T_cam_to_base[:3, 3]
        cam_look_dir = self.T_cam_to_base[:3, 2] * 0.3  # Z-axis direction, viewing direction
        ax.quiver(cam_pos[0], cam_pos[1], cam_pos[2],
                 cam_look_dir[0], cam_look_dir[1], cam_look_dir[2],
                 color='yellow', arrow_length_ratio=0.2, linewidth=3,
                 label='Camera Viewing Direction')
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_zlabel('Z (m)', fontsize=12)
        ax.set_title('Hand-Eye Calibration - 3D Spatial Layout', fontsize=16, fontweight='bold')
        ax.legend(fontsize=10, loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Set equal axis scales
        max_range = np.array([
            end_positions[:, 0].max() - end_positions[:, 0].min(),
            end_positions[:, 1].max() - end_positions[:, 1].min(),
            end_positions[:, 2].max() - end_positions[:, 2].min()
        ]).max() / 2.0
        
        mid_x = (end_positions[:, 0].max() + end_positions[:, 0].min()) * 0.5
        mid_y = (end_positions[:, 1].max() + end_positions[:, 1].min()) * 0.5
        mid_z = (end_positions[:, 2].max() + end_positions[:, 2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.tight_layout()
        output_file = Path("outputs") / "hand_eye_3d_setup.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ Saved: {output_file}")
        plt.close()
    
    def analyze_consistency(self):
        """Analyze calibration consistency"""
        print("\nAnalyzing calibration consistency...")
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        
        # Extract all pose data
        end_positions = []
        board_positions_cam = []  # Checkerboard position in camera frame
        board_positions_base = []  # Checkerboard position in base frame
        
        for data in self.calibration_data:
            T_base_to_end = np.array(data["T_base_to_end"])
            end_positions.append(T_base_to_end[:3, 3])
            
            T_cam_to_board = np.array(data["T_cam_to_board"])
            board_positions_cam.append(T_cam_to_board[:3, 3])
            
            T_base_to_board = self.T_cam_to_base @ T_cam_to_board
            board_positions_base.append(T_base_to_board[:3, 3])
        
        end_positions = np.array(end_positions)
        board_positions_cam = np.array(board_positions_cam)
        board_positions_base = np.array(board_positions_base)
        
        # 1. End-effector to checkerboard offset (should be fixed)
        ax1 = axes[0, 0]
        offsets = end_positions - board_positions_base
        offset_norms = np.linalg.norm(offsets, axis=1) * 1000  # mm
        
        ax1.bar(range(len(offset_norms)), offset_norms, color='steelblue', alpha=0.7)
        ax1.axhline(np.mean(offset_norms), color='red', linestyle='--', linewidth=2,
                   label=f'Mean Offset: {np.mean(offset_norms):.2f}mm')
        ax1.axhline(np.mean(offset_norms) + np.std(offset_norms), color='orange',
                   linestyle=':', linewidth=1.5, label=f'±1σ')
        ax1.axhline(np.mean(offset_norms) - np.std(offset_norms), color='orange',
                   linestyle=':', linewidth=1.5)
        ax1.set_xlabel('Pose Index', fontsize=12)
        ax1.set_ylabel('End-Effector to Checkerboard (mm)', fontsize=12)
        ax1.set_title('Checkerboard Mount Offset Consistency\n(Ideal: All values equal)', fontsize=13, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Checkerboard distance distribution in camera frame
        ax2 = axes[0, 1]
        distances_cam = np.linalg.norm(board_positions_cam, axis=1)
        
        ax2.scatter(range(len(distances_cam)), distances_cam, c='green', s=100, alpha=0.7)
        ax2.plot(range(len(distances_cam)), distances_cam, 'g--', alpha=0.5)
        ax2.axhline(np.mean(distances_cam), color='red', linestyle='--', linewidth=2,
                   label=f'Mean Distance: {np.mean(distances_cam):.3f}m')
        ax2.set_xlabel('Pose Index', fontsize=12)
        ax2.set_ylabel('Checkerboard to Camera (m)', fontsize=12)
        ax2.set_title('Checkerboard Distance Variation in Camera', fontsize=13, fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. XY position scatter plot (end-effector vs checkerboard)
        ax3 = axes[1, 0]
        ax3.scatter(end_positions[:, 0], end_positions[:, 1], 
                   s=150, c='blue', alpha=0.6, label='End-Effector', marker='o')
        ax3.scatter(board_positions_base[:, 0], board_positions_base[:, 1],
                   s=150, c='green', alpha=0.6, label='Checkerboard (Computed)', marker='^')
        
        # Draw connection lines
        for i in range(len(end_positions)):
            ax3.plot([end_positions[i, 0], board_positions_base[i, 0]],
                    [end_positions[i, 1], board_positions_base[i, 1]],
                    'gray', alpha=0.3, linestyle='--')
        
        ax3.set_xlabel('X (m)', fontsize=12)
        ax3.set_ylabel('Y (m)', fontsize=12)
        ax3.set_title('XY Plane Position Comparison\n(Lines = End-Effector to Checkerboard Offset)', fontsize=13, fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.set_aspect('equal', adjustable='box')
        
        # 4. Statistics
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        mean_offset = np.mean(offset_norms)
        std_offset = np.std(offset_norms)
        max_offset = np.max(offset_norms)
        min_offset = np.min(offset_norms)
        
        stats_text = f"""
        Calibration Consistency Statistics
        {'='*40}
        
        Data Points: {len(self.calibration_data)}
        
        End-Effector to Checkerboard (should be fixed):
          Mean:   {mean_offset:.2f} mm
          Std Dev: {std_offset:.2f} mm
          Max:    {max_offset:.2f} mm
          Min:    {min_offset:.2f} mm
          CV:     {(std_offset/mean_offset*100):.2f}%
        
        Checkerboard to Camera Distance:
          Mean: {np.mean(distances_cam):.3f} m
          Range: {np.min(distances_cam):.3f} - {np.max(distances_cam):.3f} m
        
        Camera Position (Base Frame):
          X: {self.cam_position[0]:.3f} m
          Y: {self.cam_position[1]:.3f} m
          Z: {self.cam_position[2]:.3f} m
        
        Camera Orientation:
          Roll:  {self.cam_euler['roll']:.1f}°
          Pitch: {self.cam_euler['pitch']:.1f}°
          Yaw:   {self.cam_euler['yaw']:.1f}°
        
        {'='*40}
        Assessment:
        """
        
        if std_offset < 5:
            stats_text += "  ✅ Excellent (Std Dev < 5mm)\n"
        elif std_offset < 10:
            stats_text += "  ✅ Good (Std Dev < 10mm)\n"
        elif std_offset < 20:
            stats_text += "  ⚠️  Fair (Std Dev < 20mm)\n"
        else:
            stats_text += "  ❌ Needs Improvement (Std Dev > 20mm)\n"
        
        ax4.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
                verticalalignment='center')
        
        plt.tight_layout()
        output_file = Path("outputs") / "hand_eye_consistency_analysis.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ Saved: {output_file}")
        plt.close()
    
    def plot_camera_view(self):
        """Plot checkerboard position distribution from camera perspective"""
        print("\nGenerating camera perspective visualization...")
        
        fig = plt.figure(figsize=(16, 6))
        
        # Extract checkerboard positions in camera frame
        board_positions_cam = []
        for data in self.calibration_data:
            T_cam_to_board = np.array(data["T_cam_to_board"])
            board_positions_cam.append(T_cam_to_board[:3, 3])
        
        board_positions_cam = np.array(board_positions_cam)
        
        # Subplot 1: XY plane (camera image plane)
        ax1 = fig.add_subplot(131)
        scatter1 = ax1.scatter(board_positions_cam[:, 0], board_positions_cam[:, 1],
                              c=board_positions_cam[:, 2], cmap='viridis',
                              s=150, alpha=0.7, edgecolors='black', linewidth=1)
        
        # Label pose indices
        for i, pos in enumerate(board_positions_cam):
            ax1.text(pos[0], pos[1], str(i), fontsize=8, ha='center', va='center')
        
        ax1.set_xlabel('X (m) - Camera Right', fontsize=11)
        ax1.set_ylabel('Y (m) - Camera Down', fontsize=11)
        ax1.set_title('Camera XY Plane View\n(Color = Depth Z)', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal', adjustable='box')
        cbar1 = plt.colorbar(scatter1, ax=ax1)
        cbar1.set_label('Depth Z (m)', fontsize=10)
        
        # Subplot 2: XZ plane
        ax2 = fig.add_subplot(132)
        scatter2 = ax2.scatter(board_positions_cam[:, 0], board_positions_cam[:, 2],
                              c=range(len(board_positions_cam)), cmap='tab20',
                              s=150, alpha=0.7, edgecolors='black', linewidth=1)
        ax2.set_xlabel('X (m)', fontsize=11)
        ax2.set_ylabel('Z (m) - Depth', fontsize=11)
        ax2.set_title('Camera XZ Plane View', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal', adjustable='box')
        
        # Subplot 3: YZ plane
        ax3 = fig.add_subplot(133)
        scatter3 = ax3.scatter(board_positions_cam[:, 1], board_positions_cam[:, 2],
                              c=range(len(board_positions_cam)), cmap='tab20',
                              s=150, alpha=0.7, edgecolors='black', linewidth=1)
        ax3.set_xlabel('Y (m)', fontsize=11)
        ax3.set_ylabel('Z (m) - Depth', fontsize=11)
        ax3.set_title('Camera YZ Plane View', fontsize=12, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.set_aspect('equal', adjustable='box')
        
        plt.tight_layout()
        output_file = Path("outputs") / "hand_eye_camera_view.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ Saved: {output_file}")
        plt.close()
    
    def generate_all_visualizations(self):
        """Generate all visualizations"""
        print("\n" + "="*60)
        print("Hand-Eye Calibration Result Visualization")
        print("="*60)
        
        self.plot_3d_setup()
        self.analyze_consistency()
        self.plot_camera_view()
        
        print("\n" + "="*60)
        print("✅ All visualizations completed!")
        print("="*60)
        print("\nGenerated files:")
        print("  1. hand_eye_3d_setup.png - 3D Spatial Layout")
        print("  2. hand_eye_consistency_analysis.png - Consistency Analysis")
        print("  3. hand_eye_camera_view.png - Camera Perspective")
        print("\n💡 Tips:")
        print("  - Good calibration: End-effector and checkerboard should have fixed offset")
        print("  - Offset std dev < 10mm indicates good calibration quality")
        print("  - Checkerboard should be distributed across different regions in camera view")


def main():
    """Main program"""
    try:
        visualizer = HandEyeVisualizer()
        visualizer.generate_all_visualizations()
        
    except FileNotFoundError as e:
        print(f"\n❌ Error: File not found")
        print(f"   {e}")
        print("\nPlease ensure:")
        print("   1. Hand-eye calibration data collected (hand_eye_data.json)")
        print("   2. Hand-eye calibration solved (camera_to_base_calibration.json)")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
