#!/usr/bin/env python3
"""
Direct FK-based Table Height Calibration
=========================================

This script measures the table height by:
1. Manually moving the stylus tip to touch the table surface
2. Reading current joint angles
3. Computing FK to get stylus tip position
4. Recording the Z coordinate as table height
"""

import numpy as np
import json
from pathlib import Path

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics


def main():
    """Main calibration function"""
    print("\n" + "="*60)
    print("Table Height Calibration - Direct FK Method")
    print("="*60)
    
    # Initialize robot
    print("\nInitializing robot...")
    robot_config = SO101FollowerConfig(
        port="/dev/ttyACM0",
        id="hand_eye_calib_arm",  # Use same ID as hand_eye calibration to share calibration file
    )
    robot = SO101Follower(robot_config)
    
    # Initialize kinematics
    print("Initializing kinematics solver...")
    kinematics = RobotKinematics(
        urdf_path="SO101/so101_5dof_stylus.urdf",
        target_frame_name="stylus_tip_link",  # FK calculates stylus tip position
        joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex",
                    "wrist_flex", "wrist_roll"]
    )
    
    # Connect robot (will use existing calibration file)
    print("Connecting to robot...")
    robot.connect()
    
    print("\n" + "="*60)
    print("CALIBRATION INSTRUCTIONS")
    print("="*60)
    print("\n1. The robot torque will be DISABLED")
    print("2. Manually move the robot so the stylus tip touches the table surface")
    print("3. Make sure the stylus tip is in FIRM CONTACT with the table")
    print("4. Press ENTER when ready to record the position")
    print("\nNote: The stylus tip position will be calculated using FK")
    print("="*60)
    
    # Disable torque to allow manual movement
    print("\nDisabling robot torque...")
    robot.bus.disable_torque()
    print("✅ Torque disabled - You can now move the robot manually")
    
    # Wait for user to position the robot
    input("\nPress ENTER when the stylus tip is touching the table surface: ")
    
    # Read current joint positions
    print("\nReading joint positions...")
    obs = robot.get_observation()
    joint_positions = np.array([
        obs["shoulder_pan.pos"],
        obs["shoulder_lift.pos"],
        obs["elbow_flex.pos"],
        obs["wrist_flex.pos"],
        obs["wrist_roll.pos"],
    ])
    
    print(f"  Joint positions (degrees): {joint_positions}")
    
    # Compute FK to get stylus tip position
    print("\nComputing forward kinematics...")
    stylus_tip_pose = kinematics.forward_kinematics(joint_positions)
    stylus_tip_pos = stylus_tip_pose[:3, 3]  # Extract position (x, y, z)
    
    print(f"  Stylus tip position in base frame:")
    print(f"    X: {stylus_tip_pos[0]*1000:.1f} mm")
    print(f"    Y: {stylus_tip_pos[1]*1000:.1f} mm")
    print(f"    Z: {stylus_tip_pos[2]*1000:.1f} mm")
    
    # The Z coordinate is the table height
    table_height = stylus_tip_pos[2]
    
    print("\n" + "="*60)
    print("CALIBRATION RESULT")
    print("="*60)
    print(f"\nTable height (Z coordinate): {table_height*1000:.1f} mm = {table_height:.4f} m")
    print(f"  (This is the Z coordinate of the table surface in the robot base frame)")
    
    # Save result
    output_dir = Path("outputs")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / "table_height.json"
    
    result = {
        "table_height_base": float(table_height),
        "table_height_mm": float(table_height * 1000),
        "measurement_method": "direct_fk",
        "joint_positions_deg": joint_positions.tolist(),
        "stylus_tip_position_m": {
            "x": float(stylus_tip_pos[0]),
            "y": float(stylus_tip_pos[1]),
            "z": float(stylus_tip_pos[2])
        }
    }
    
    with open(output_file, 'w') as f:
        json.dump(result, f, indent=2)
    
    print(f"\n✅ Result saved to: {output_file}")
    
    # Re-enable torque
    print("\nRe-enabling robot torque...")
    robot.bus.enable_torque()
    print("✅ Torque enabled")
    
    # Disconnect
    print("\nDisconnecting robot...")
    robot.disconnect()
    print("✅ Disconnected")
    
    print("\n" + "="*60)
    print("Calibration complete!")
    print("="*60)
    print("\nYou can now use this table height for:")
    print("  - Calculating target Z coordinates for touch positions")
    print("  - Converting 2D image coordinates to 3D positions")
    print("  - Ensuring the stylus doesn't go below the table surface")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n❌ Calibration cancelled by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
