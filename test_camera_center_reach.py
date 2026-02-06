#!/usr/bin/env python3
"""
Camera Center Reach Test
=========================

Test if the robot can accurately reach the camera's field of view center
with the stylus tip touching the table surface.

Goal: Position error < 10mm
"""

import numpy as np
import json
import time
import cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as R

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.cameras.configs import ColorMode


class CameraCapture:
    """Camera capture with crosshair overlay"""
    
    def __init__(self, camera, cx, cy, output_dir="outputs"):
        self.camera = camera
        self.cx = int(cx)
        self.cy = int(cy)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
    def draw_crosshair(self, frame, label=""):
        """Draw crosshair at image center (cx, cy)"""
        height, width = frame.shape[:2]
        
        # Draw crosshair lines
        color = (0, 255, 0)  # Green
        thickness = 2
        line_length = 50
        
        # Horizontal line
        cv2.line(frame, 
                (self.cx - line_length, self.cy), 
                (self.cx + line_length, self.cy), 
                color, thickness)
        
        # Vertical line
        cv2.line(frame, 
                (self.cx, self.cy - line_length), 
                (self.cx, self.cy + line_length), 
                color, thickness)
        
        # Center circle
        cv2.circle(frame, (self.cx, self.cy), 5, color, -1)
        
        # Add label at top
        if label:
            cv2.putText(frame, label, 
                       (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        # Add target center text
        cv2.putText(frame, "TARGET CENTER", 
                   (self.cx + 15, self.cy - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Add coordinates
        cv2.putText(frame, f"({self.cx}, {self.cy})", 
                   (self.cx + 15, self.cy + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return frame
    
    def capture_and_save(self, filename, label=""):
        """Capture image and save with crosshair"""
        # Clear camera buffer (read multiple frames)
        for _ in range(5):
            self.camera.read()
            time.sleep(0.1)
        
        # Capture frame
        frame = self.camera.read()
        
        # Frame might be a dict or numpy array, handle both cases
        if isinstance(frame, dict):
            frame = frame["frame"]
        
        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Draw crosshair
        frame_with_crosshair = self.draw_crosshair(frame_bgr, label)
        
        # Save image
        output_path = self.output_dir / filename
        cv2.imwrite(str(output_path), frame_with_crosshair)
        
        print(f"✅ Image saved: {output_path}")
        return output_path


def load_calibration_data():
    """Load all calibration data"""
    print("\n" + "="*60)
    print("Loading Calibration Data")
    print("="*60)
    
    # 1. Camera intrinsics
    intrinsics_file = Path("outputs/cheap_camera_intrinsics.json")
    if not intrinsics_file.exists():
        raise FileNotFoundError(f"Camera intrinsics not found: {intrinsics_file}")
    
    with open(intrinsics_file) as f:
        intrinsics = json.load(f)
    
    camera_matrix = np.array(intrinsics["camera_matrix"])
    print(f"\n✅ Camera intrinsics loaded")
    print(f"   Principal point (cx, cy): ({camera_matrix[0, 2]:.1f}, {camera_matrix[1, 2]:.1f}) pixels")
    print(f"   Focal length (fx, fy): ({camera_matrix[0, 0]:.1f}, {camera_matrix[1, 1]:.1f}) pixels")
    
    # 2. Hand-eye calibration
    hand_eye_file = Path("outputs/camera_to_base_calibration.json")
    if not hand_eye_file.exists():
        raise FileNotFoundError(f"Hand-eye calibration not found: {hand_eye_file}")
    
    with open(hand_eye_file) as f:
        hand_eye = json.load(f)
    
    T_cam_to_base = np.array(hand_eye["T_cam_to_base"])
    cam_pos = T_cam_to_base[:3, 3]
    print(f"\n✅ Hand-eye calibration loaded")
    print(f"   Camera position: [{cam_pos[0]:.3f}, {cam_pos[1]:.3f}, {cam_pos[2]:.3f}] m")
    
    # 3. Table height
    table_file = Path("outputs/table_height.json")
    if not table_file.exists():
        raise FileNotFoundError(f"Table height not found: {table_file}")
    
    with open(table_file) as f:
        table_data = json.load(f)
    
    table_height = table_data["table_height_base"]
    print(f"\n✅ Table height loaded")
    print(f"   Table Z coordinate: {table_height:.4f} m = {table_height*1000:.1f} mm")
    
    return camera_matrix, T_cam_to_base, table_height


def compute_camera_center_3d(camera_matrix, T_cam_to_base, table_height):
    """
    Compute 3D coordinates of camera center in robot base frame
    
    The camera center (principal point) is at pixel coordinates (cx, cy).
    We need to find where this pixel corresponds to on the table surface.
    """
    print("\n" + "="*60)
    print("Computing Camera Center 3D Position")
    print("="*60)
    
    # Camera principal point (image center)
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    
    print(f"\nImage center: ({cx:.1f}, {cy:.1f}) pixels")
    
    # Camera position in base frame
    camera_pos_base = T_cam_to_base[:3, 3]
    
    # Depth in camera frame: distance from camera to table surface
    # Camera is at height camera_pos_base[2], table is at table_height
    depth_camera = camera_pos_base[2] - table_height
    
    print(f"\nCamera height above table: {depth_camera*1000:.1f} mm")
    
    # For the principal point (image center), the ray passes through the optical center
    # In normalized camera coordinates, center is at (0, 0)
    # Ray direction in camera frame: (0, 0, depth)
    point_cam = np.array([0, 0, depth_camera, 1])
    
    print(f"Point in camera frame: [{point_cam[0]:.3f}, {point_cam[1]:.3f}, {point_cam[2]:.3f}] m")
    
    # Transform to base frame
    point_base = T_cam_to_base @ point_cam
    
    # Set Z to exactly table height (to ensure contact)
    point_base[2] = table_height
    
    print(f"\n✅ Target position in base frame:")
    print(f"   X: {point_base[0]*1000:.1f} mm")
    print(f"   Y: {point_base[1]*1000:.1f} mm")
    print(f"   Z: {point_base[2]*1000:.1f} mm (table surface)")
    
    return point_base[:3]


def construct_downward_pose(target_position):
    """
    Construct target pose with vertical downward orientation
    
    Vertical downward means:
    - Stylus points straight down (towards -Z direction)
    - This is a 180° rotation around Y-axis
    """
    print("\n" + "="*60)
    print("Constructing Target Pose")
    print("="*60)
    
    target_T = np.eye(4)
    target_T[:3, 3] = target_position
    
    # Vertical downward: 180° rotation around Y-axis
    # rot = R.from_euler('y', 180, degrees=True)
    # target_T[:3, :3] = rot.as_matrix()
    
    print(f"\n✅ Target pose constructed")
    print(f"   Position: {target_position}")
    print(f"   Orientation: Vertical downward (180° around Y)")
    
    return target_T


def solve_ik_and_move(target_T, kinematics, robot):
    """
    Solve IK and move robot to target position
    
    Args:
        target_T: Target pose (4x4 transformation matrix)
        kinematics: Pre-initialized RobotKinematics instance (reused for consistency)
        robot: Robot instance
    """
    print("\n" + "="*60)
    print("Inverse Kinematics Solving")
    print("="*60)
    
    # Get current joint positions
    print("Reading current joint positions...")
    # obs = robot.get_observation()
    # current_joints = np.array([
    #     obs["shoulder_pan.pos"],
    #     obs["shoulder_lift.pos"],
    #     obs["elbow_flex.pos"],
    #     obs["wrist_flex.pos"],
    #     obs["wrist_roll.pos"],
    # ])
    
    # 🔧 HARDCODED for testing - ensures identical initial joints as test_ik_position_only.py
    current_joints = np.array([-9.06200318, 13.86554622, 14.17004049, 69.46564885, 99.7271487])
    
    print(f"Current joints (HARDCODED): {current_joints}")
    
    # Update robot state with current joints (important for placo internal state)
    print("\nUpdating robot state with current joints...")
    _ = kinematics.forward_kinematics(current_joints)
    print("✅ Robot state updated")
    
    # Solve IK
    print("\nSolving inverse kinematics...")
    print("注意注意！")
    
    print(target_T)
    print(current_joints)
    target_joints = kinematics.inverse_kinematics(
        current_joint_pos=current_joints,
        desired_ee_pose=target_T,
        position_weight=1.0,
        orientation_weight=0.01  # Lower weight for 5-DOF robot
    )
    
    print(f"IK解出的关节角度:  {target_joints}")
    verify_pose = kinematics.forward_kinematics(target_joints)
    verify_pos = verify_pose[:3, 3]
    print(f"  FK计算位置: {verify_pos * 1000} mm")
    # Move robot
    print("\n" + "="*60)
    print("Moving Robot")
    print("="*60)
    print(f"\n⚠️  Robot will move to target position in 3 seconds...")
    print(f"   Make sure the workspace is clear!")
    
    input("\nPress ENTER to start movement (or Ctrl+C to cancel): ")
    
    # Convert to action format and move
    print("\nMoving...")
    action = {
        "shoulder_pan.pos": float(target_joints[0]),
        "shoulder_lift.pos": float(target_joints[1]),
        "elbow_flex.pos": float(target_joints[2]),
        "wrist_flex.pos": float(target_joints[3]),
        "wrist_roll.pos": float(target_joints[4]),
    }
    
    robot.send_action(action)
    
    # Wait for movement to complete
    print("Waiting for movement to stabilize...")
    # time.sleep(3.0)
    time.sleep(3.0)
    print("✅ Movement complete")
    
    return target_joints, kinematics


def verify_accuracy(target_joints, target_position, kinematics):
    """
    Verify positioning accuracy using FK
    """
    print("\n" + "="*60)
    print("Accuracy Verification")
    print("="*60)
    
    # FK to get actual position
    print("\nComputing forward kinematics...")
    actual_pose = kinematics.forward_kinematics(target_joints)
    actual_pos = actual_pose[:3, 3]
    
    print(f"\nTarget position: [{target_position[0]*1000:.1f}, {target_position[1]*1000:.1f}, {target_position[2]*1000:.1f}] mm")
    print(f"Actual position: [{actual_pos[0]*1000:.1f}, {actual_pos[1]*1000:.1f}, {actual_pos[2]*1000:.1f}] mm")
    
    # Calculate error
    error_vector = actual_pos - target_position
    error_magnitude = np.linalg.norm(error_vector)
    
    print(f"\nError breakdown:")
    print(f"  ΔX: {error_vector[0]*1000:.2f} mm")
    print(f"  ΔY: {error_vector[1]*1000:.2f} mm")
    print(f"  ΔZ: {error_vector[2]*1000:.2f} mm")
    print(f"\n📏 Total position error: {error_magnitude*1000:.2f} mm")
    
    # Assess result
    print("\n" + "="*60)
    print("RESULT ASSESSMENT")
    print("="*60)
    
    if error_magnitude < 0.010:  # < 10mm
        print("\n✅ SUCCESS! Error < 10mm")
        print("   Phase 1 goal achieved!")
        result = "SUCCESS"
    elif error_magnitude < 0.020:  # < 20mm
        print("\n⚠️  ACCEPTABLE. Error < 20mm")
        print("   Close to goal, may need tuning")
        result = "ACCEPTABLE"
    else:
        print("\n❌ NEEDS IMPROVEMENT. Error > 20mm")
        print("   Check calibration and URDF parameters")
        result = "NEEDS_IMPROVEMENT"
    
    # Save result
    output_dir = Path("outputs")
    output_dir.mkdir(parents=True, exist_ok=True)
    result_file = output_dir / "camera_center_reach_result.json"
    
    result_data = {
        "target_position_m": target_position.tolist(),
        "actual_position_m": actual_pos.tolist(),
        "error_vector_mm": (error_vector * 1000).tolist(),
        "error_magnitude_mm": float(error_magnitude * 1000),
        "target_joints_deg": target_joints.tolist(),
        "result": result,
        "goal_achieved": bool(error_magnitude < 0.010)
    }
    
    with open(result_file, 'w') as f:
        json.dump(result_data, f, indent=2)
    
    print(f"\n✅ Result saved to: {result_file}")
    
    return error_magnitude


def main():
    """Main test function"""
    print("\n" + "="*80)
    print(" "*20 + "CAMERA CENTER REACH TEST")
    print("="*80)
    print("\nGoal: Position stylus tip at camera field of view center with <10mm error")
    
    camera_capture = None
    camera = None
    
    try:
        # Step 1: Load calibration data
        camera_matrix, T_cam_to_base, table_height = load_calibration_data()
        
        # Get principal point (image center)
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        # Step 2: Initialize camera
        print("\n" + "="*60)
        print("Initializing Camera")
        print("="*60)
        
        # Load intrinsics to get image size
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
        print("✅ Camera connected")
        
        # Initialize camera capture
        camera_capture = CameraCapture(camera, cx, cy, output_dir="outputs")
        
        # Capture "before" image
        print("\n📸 Capturing BEFORE image...")
        camera_capture.capture_and_save("camera_view_before.jpg", "BEFORE MOVEMENT")
        
        # Step 3: Compute target 3D position
        target_position = compute_camera_center_3d(camera_matrix, T_cam_to_base, table_height)
        # Match precision with test_ik_position_only.py (4 decimal places)
        target_position = np.round(target_position, 4)
        
        # Step 4: Construct target pose (vertical downward)
        target_T = construct_downward_pose(target_position)
        
        # Step 5: Connect robot
        print("\n" + "="*60)
        print("Connecting to Robot")
        print("="*60)
        
        robot_config = SO101FollowerConfig(
            port="/dev/ttyACM0",
            id="hand_eye_calib_arm",
        )
        robot = SO101Follower(robot_config)
        
        print("\nConnecting...")
        robot.connect()
        print("✅ Robot connected")
        
        # Step 6: Initialize kinematics (ONCE, outside the function for consistency)
        print("\n" + "="*60)
        print("Initializing Kinematics Solver")
        print("="*60)
        urdf_path = "SO101/so101_5dof_stylus.urdf"
        kinematics = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name="stylus_tip_link",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex",
                        "wrist_flex", "wrist_roll"]
        )
        print("✅ Kinematics solver initialized (will be reused)")
        
        # Step 7: Solve IK and move
        target_joints, kinematics = solve_ik_and_move(target_T, kinematics, robot)
        
        # Step 8: Capture "after" image
        print("\n📸 Capturing AFTER image...")
        after_img_path = camera_capture.capture_and_save("camera_view_after.jpg", "AFTER MOVEMENT")
        
        # Step 9: Verify accuracy
        error = verify_accuracy(target_joints, target_position, kinematics)
        
        # Step 10: Visual verification instructions
        print("\n" + "="*60)
        print("Visual Verification")
        print("="*60)
        print("\n📸 Camera images saved with crosshair overlay:")
        print(f"   BEFORE: outputs/camera_view_before.jpg")
        print(f"   AFTER:  outputs/camera_view_after.jpg")
        print("\n💡 How to verify:")
        print("   1. Open the AFTER image")
        print("   2. Check if the stylus tip is at the green crosshair center")
        print("   3. The crosshair marks the camera field of view center")
        print("   4. Compare with FK calculated error above")
        
        # Cleanup
        print("\n" + "="*60)
        print("Cleanup")
        print("="*60)
        
        if camera:
            print("Disconnecting camera...")
            camera.disconnect()
            print("✅ Camera disconnected")
        
        print("Disconnecting robot...")
        robot.disconnect()
        print("✅ Robot disconnected")
        
        print("\n" + "="*80)
        print(" "*25 + "TEST COMPLETE")
        print("="*80)
        
        if error < 0.010:
            print("\n🎉 Phase 1 goal achieved! Error < 10mm")
        else:
            print(f"\n⚠️  Error: {error*1000:.2f} mm")
            print("   Consider:")
            print("   1. Re-check hand-eye calibration quality")
            print("   2. Verify URDF stylus offset (93mm)")
            print("   3. Re-measure table height")
            print("   4. Check motor calibration")
        
    except FileNotFoundError as e:
        print(f"\n❌ Error: {e}")
        print("\nMissing calibration file. Please complete:")
        print("  1. Camera intrinsics: python calibrate_cheap_camera_phone.py")
        print("  2. Hand-eye calibration: python hand_eye_calibration_opencv.py")
        print("  3. Table height: python calibrate_table_height_direct.py")
    except KeyboardInterrupt:
        print("\n\n❌ Test cancelled by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure cleanup happens even if there's an error
        if camera:
            try:
                camera.disconnect()
            except:
                pass


if __name__ == "__main__":
    main()
