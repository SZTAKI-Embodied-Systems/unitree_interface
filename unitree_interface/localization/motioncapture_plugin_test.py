"""
Test script for OptiTrack motion capture plugin
Visualizes the robot in MuJoCo using real-time OptiTrack data
"""
import os
import sys
import time
import json
import numpy as np
from pathlib import Path
import mujoco
from mujoco import viewer
from scipy.spatial.transform import Rotation as R

from dial_mpc.deploy.localization.motioncapture_plugin import OptiTrackPlugin


def quaternion_to_euler_numpy(q):
    """
    Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw)
    """
    w, x, y, z = q
    # Rotation matrix
    rot_matrix = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    
    # Euler angles
    pitch = -np.arcsin(np.clip(rot_matrix[2, 0], -1.0, 1.0))
    roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
    yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    
    return roll, pitch, yaw


def create_go2_model():
    """
    Create a simple Go2 robot model for visualization
    Uses the unitree_go2 model from dial_mpc
    """
    # Try to find the model in the workspace
    model_path = Path(__file__).parents[2] / "models" / "unitree_go2" / "scene.xml"
    
    if not model_path.exists():
        raise FileNotFoundError(f"Could not find Go2 model at {model_path}")
    
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    
    return model, data


def test_motioncapture_plugin(
    hostname="192.168.2.141",
    object_name="go2",
    z_offset=0.0,
    duration=60.0,
    save_data=True,
    output_path=None
):
    """
    Test the OptiTrack motion capture plugin with MuJoCo visualization
    
    Args:
        hostname: OptiTrack server hostname/IP
        object_name: Name of the rigid body to track
        z_offset: Z-axis offset for the tracked object
        duration: Test duration in seconds (0 for infinite)
        save_data: Whether to save logged data
        output_path: Path to save logged data (default: data/motioncapture_test/)
    """
    print("=" * 60)
    print("OptiTrack Motion Capture Plugin Test")
    print("=" * 60)
    
    # Configure the plugin
    config = {
        'optitrack_hostname': hostname,
        'optitrack_object_name': object_name,
        'optitrack_z_offset': z_offset
    }
    
    print(f"\nConfiguration:")
    print(f"  Hostname: {hostname}")
    print(f"  Object: {object_name}")
    print(f"  Z-offset: {z_offset}")
    print(f"  Duration: {duration}s" if duration > 0 else "  Duration: Infinite (Ctrl+C to stop)")
    
    # Initialize the plugin
    print("\nInitializing OptiTrack plugin...")
    try:
        plugin = OptiTrackPlugin(config)
        print("✓ Plugin initialized successfully")
    except Exception as e:
        print(f"✗ Failed to initialize plugin: {e}")
        return
    
    # Wait for first data
    print("\nWaiting for first OptiTrack frame...")
    time.sleep(0.5)
    
    # Create MuJoCo model
    print("Loading MuJoCo model...")
    try:
        model, data = create_go2_model()
        print(f"✓ Model loaded: {model.nq} DOFs")
    except Exception as e:
        print(f"✗ Failed to load model: {e}")
        print("  Continuing without visualization...")
        model, data = None, None
    
    # Data logging
    log_data = {
        "time": [],
        "position": [],
        "quaternion": [],
        "linear_velocity": [],
        "angular_velocity": [],
        "euler_angles": []
    }
    
    # Main test loop
    print("\n" + "=" * 60)
    print("Starting data acquisition...")
    print("=" * 60)
    print("\nControls:")
    print("  Ctrl+C: Stop test and save data")
    if model is not None:
        print("  Mouse: Rotate/pan/zoom view")
    print()
    
    start_time = time.time()
    frame_count = 0
    last_print_time = start_time
    
    try:
        if model is not None:
            # Visualization mode
            with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=False) as viewer:
                # Enable coordinate frame visualization
                viewer.opt.frame = mujoco.mjtFrame.mjFRAME_WORLD
                
                while viewer.is_running():
                    current_time = time.time() - start_time
                    
                    # Check duration
                    if duration > 0 and current_time >= duration:
                        print(f"\n✓ Test completed ({duration}s)")
                        break
                    
                    # Get state from plugin
                    try:
                        state = plugin.get_state()
                        update_time = plugin.get_last_update_time()
                        
                        if state is not None:
                            # Extract state components
                            position = state[0:3]
                            quaternion = state[3:7]  # wxyz format from plugin
                            linear_velocity = state[7:10]
                            angular_velocity = state[10:13]
                            
                            # Convert quaternion to euler for logging/display only
                            roll, pitch, yaw = quaternion_to_euler_numpy(quaternion)
                            
                            # Update MuJoCo visualization - use quaternion directly
                            data.qpos[0:3] = position
                            # MuJoCo uses wxyz quaternion format, which matches our plugin output
                            data.qpos[3:7] = quaternion
                            # Keep default joint positions for now
                            data.qvel[0:3] = linear_velocity
                            data.qvel[3:6] = angular_velocity
                            
                            mujoco.mj_forward(model, data)
                            viewer.sync()
                            
                            # Log data
                            if save_data:
                                log_data["time"].append(current_time)
                                log_data["position"].append(position.tolist())
                                log_data["quaternion"].append(quaternion.tolist())
                                log_data["linear_velocity"].append(linear_velocity.tolist())
                                log_data["angular_velocity"].append(angular_velocity.tolist())
                                log_data["euler_angles"].append([roll, pitch, yaw])
                            
                            frame_count += 1
                            
                            # Print status every second
                            if current_time - last_print_time >= 1.0:
                                print(f"[{current_time:6.1f}s] Frames: {frame_count:4d} | "
                                      f"Pos: [{position[0]:6.3f}, {position[1]:6.3f}, {position[2]:6.3f}] | "
                                      f"RPY: [{roll:6.3f}, {pitch:6.3f}, {yaw:6.3f}]")
                                last_print_time = current_time
                    
                    except Exception as e:
                        print(f"Error reading state: {e}")
                    
                    # Small sleep to avoid busy loop
                    time.sleep(0.01)
        else:
            # No visualization mode - just log data
            while True:
                current_time = time.time() - start_time
                
                # Check duration
                if duration > 0 and current_time >= duration:
                    print(f"\n✓ Test completed ({duration}s)")
                    break
                
                # Get state from plugin
                try:
                    state = plugin.get_state()
                    
                    if state is not None:
                        # Extract state components
                        position = state[0:3]
                        quaternion = state[3:7]
                        linear_velocity = state[7:10]
                        angular_velocity = state[10:13]
                        
                        # Convert quaternion to euler for display
                        roll, pitch, yaw = quaternion_to_euler_numpy(quaternion)
                        
                        # Log data
                        if save_data:
                            log_data["time"].append(current_time)
                            log_data["position"].append(position.tolist())
                            log_data["quaternion"].append(quaternion.tolist())
                            log_data["linear_velocity"].append(linear_velocity.tolist())
                            log_data["angular_velocity"].append(angular_velocity.tolist())
                            log_data["euler_angles"].append([roll, pitch, yaw])
                        
                        frame_count += 1
                        
                        # Print status every second
                        if current_time - last_print_time >= 1.0:
                            print(f"[{current_time:6.1f}s] Frames: {frame_count:4d} | "
                                  f"Pos: [{position[0]:6.3f}, {position[1]:6.3f}, {position[2]:6.3f}] | "
                                  f"RPY: [{roll:6.3f}, {pitch:6.3f}, {yaw:6.3f}]")
                            last_print_time = current_time
                
                except Exception as e:
                    print(f"Error reading state: {e}")
                
                # Sleep to match desired rate
                time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n\n✓ Test stopped by user")
    
    # Save logged data
    if save_data and frame_count > 0:
        if output_path is None:
            output_path = Path("output/motioncapture_test")
        else:
            output_path = Path(output_path)
        
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Save as JSON
        json_path = output_path / "log_optitrack.json"
        with open(json_path, "w") as f:
            json.dump(log_data, f, indent=2)
        
        print(f"\n✓ Saved {frame_count} frames to {json_path}")
        
        # Print statistics
        positions = np.array(log_data["position"])
        velocities = np.array(log_data["linear_velocity"])
        
        print("\nStatistics:")
        print(f"  Duration: {log_data['time'][-1]:.2f}s")
        print(f"  Frames: {frame_count}")
        print(f"  Avg rate: {frame_count / log_data['time'][-1]:.1f} Hz")
        print(f"\n  Position range:")
        print(f"    X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
        print(f"    Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
        print(f"    Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
        print(f"\n  Velocity range:")
        print(f"    vX: [{velocities[:, 0].min():.3f}, {velocities[:, 0].max():.3f}] m/s")
        print(f"    vY: [{velocities[:, 1].min():.3f}, {velocities[:, 1].max():.3f}] m/s")
        print(f"    vZ: [{velocities[:, 2].min():.3f}, {velocities[:, 2].max():.3f}] m/s")
    
    print("\n" + "=" * 60)
    print("Test completed")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test OptiTrack motion capture plugin")
    parser.add_argument("--hostname", type=str, default="192.168.2.141",
                        help="OptiTrack server hostname/IP")
    parser.add_argument("--object", type=str, default="go2",
                        help="Rigid body name to track")
    parser.add_argument("--z-offset", type=float, default=0.0,
                        help="Z-axis offset for tracked object")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Test duration in seconds (0 for infinite)")
    parser.add_argument("--no-save", action="store_true",
                        help="Disable data logging")
    parser.add_argument("--output", type=str, default=None,
                        help="Output directory for logged data")
    
    args = parser.parse_args()
    
    test_motioncapture_plugin(
        hostname=args.hostname,
        object_name=args.object,
        z_offset=args.z_offset,
        duration=args.duration,
        save_data=not args.no_save,
        output_path=args.output
    )
