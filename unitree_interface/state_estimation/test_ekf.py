import json
import pickle
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import time
from unitree_interface.state_estimation.ekf import EKF


def normalize_quat_wxyz(quat_wxyz):
    quat = np.asarray(quat_wxyz, dtype=float).reshape(4)
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return quat / norm


def yaw_from_quat_wxyz(quat_wxyz):
    w, x, y, z = normalize_quat_wxyz(quat_wxyz)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def main():
    log_path = Path("unitree_interface/data/log_robot.json")
    if not log_path.exists():
        raise FileNotFoundError(f"Log file not found: {log_path}")
    with log_path.open("r") as f:
        log_data = json.load(f)

    opti_position = np.asarray(log_data["raw_opti_position"], dtype=float)
    opti_rotation = np.asarray(log_data["raw_opti_quat_wxyz"], dtype=float)  # expected [w, x, y, z]
    opti_velocity = np.asarray(log_data["raw_opti_velocity"], dtype=float)  # expected [x, y, z]
    joint_position = np.asarray(log_data["raw_joint_position"], dtype=float)  # 12 joints
    joint_velocity = np.asarray(log_data["raw_joint_velocity"], dtype=float)  # 12 joint velocities
    gyro_velocity = np.asarray(log_data["raw_gyro_body"], dtype=float)  # 12 joint velocities
    t = np.asarray(log_data["time"], dtype=float)

    n = min(len(opti_position), len(opti_rotation), len(joint_position), len(joint_velocity), len(t))
    if n < 3:
        raise RuntimeError("Not enough samples in log file to run EKF replay test.")
    
    print(f"Total samples to process: {n}\n")

    ekf = EKF()

    ekf.initialize(
        position=opti_position[0],
        velocity=opti_velocity[0],
        quat_wxyz=opti_rotation[0],
        joint_pos=joint_position[0],
        joint_vel=joint_velocity[0],
    )

    pos_filt_sq = []
    yaw_filt_sq = []
    joint_pos_sq = []
    joint_vel_sq = []
    
    # Store data for plotting
    time_vec = []
    pos_meas_list = []
    pos_filt_list = []
    vel_meas_list = []
    vel_filt_list = []
    yaw_meas_list = []
    yaw_filt_list = []
    joint_pos_meas_list = []
    joint_pos_filt_list = []
    joint_vel_meas_list = []
    joint_vel_filt_list = []
    gyro_bias_list = []
    quat_filt_list = []
    ekf_step_times = []

    t_wall_start = time.perf_counter()

    for k in range(1, n):
        dt = max(float(t[k] - t[k - 1]), 1e-4)

        t_ekf_step_start = time.perf_counter()
        ekf.predict(gyro_body=gyro_velocity[k], dt=dt)
        ekf.update_optitrack(
            position=opti_position[k],
            velocity=opti_velocity[k],
            quat_wxyz=opti_rotation[k],
        )
        ekf.update_joints(
            joint_pos=joint_position[k],
            joint_vel=joint_velocity[k],
        )
        ekf_step_times.append(time.perf_counter() - t_ekf_step_start)

        pos_f, vel_f, quat_f, bg_f, joint_pos_f, joint_vel_f = ekf.get_state()

        pos_filt_err = np.linalg.norm(pos_f - opti_position[k])
        pos_filt_sq.append(pos_filt_err**2)

        yaw_meas = yaw_from_quat_wxyz(opti_rotation[k])
        yaw_est = yaw_from_quat_wxyz(quat_f)
        yaw_diff = np.arctan2(np.sin(yaw_meas - yaw_est), np.cos(yaw_meas - yaw_est))
        yaw_filt_sq.append(yaw_diff**2)

        joint_pos_err = np.linalg.norm(joint_pos_f - joint_position[k])
        joint_vel_err = np.linalg.norm(joint_vel_f - joint_velocity[k])
        joint_pos_sq.append(joint_pos_err**2)
        joint_vel_sq.append(joint_vel_err**2)
        
        # Collect data for plotting
        time_vec.append(t[k])
        pos_meas_list.append(opti_position[k].copy())
        pos_filt_list.append(pos_f.copy())
        vel_filt_list.append(vel_f.copy())
        vel_meas_list.append(opti_velocity[k].copy())
        yaw_meas_list.append(yaw_meas)
        yaw_filt_list.append(yaw_est)
        joint_pos_meas_list.append(joint_position[k].copy())
        joint_pos_filt_list.append(joint_pos_f.copy())
        joint_vel_meas_list.append(joint_velocity[k].copy())
        joint_vel_filt_list.append(joint_vel_f.copy())
        gyro_bias_list.append(bg_f.copy())
        quat_filt_list.append(quat_f.copy())

        if k % 500 == 0:
            print(
                f"Step {k:4d}/{n-1} | dt={dt:.4f}s | "
                f"|p_err|={pos_filt_err:.4f}m | yaw_err={np.rad2deg(abs(yaw_diff)):.3f}deg | "
                f"|q_joint_err|={joint_pos_err:.5f}rad | |dq_joint_err|={joint_vel_err:.4f}rad/s"
            )

    total_wall_time = time.perf_counter() - t_wall_start
    ekf_step_times = np.array(ekf_step_times, dtype=float)
    ekf_total_time = float(np.sum(ekf_step_times))
    ekf_mean_time = float(np.mean(ekf_step_times)) if len(ekf_step_times) > 0 else 0.0
    ekf_min_time = float(np.min(ekf_step_times)) if len(ekf_step_times) > 0 else 0.0
    ekf_max_time = float(np.max(ekf_step_times)) if len(ekf_step_times) > 0 else 0.0
    ekf_mean_rate_hz = (1.0 / ekf_mean_time) if ekf_mean_time > 0.0 else 0.0

    pos_tracking_rmse = np.sqrt(np.mean(pos_filt_sq))
    yaw_tracking_rmse_deg = np.rad2deg(np.sqrt(np.mean(yaw_filt_sq)))
    joint_pos_rmse = np.sqrt(np.mean(joint_pos_sq))
    joint_vel_rmse = np.sqrt(np.mean(joint_vel_sq))

    print("\nGo2FullStateEKF log replay finished")
    print(f"Samples used: {n}")
    print(f"EKF base position tracking RMSE vs OptiTrack: {pos_tracking_rmse:.4f} m")
    print(f"EKF base yaw tracking RMSE vs OptiTrack: {yaw_tracking_rmse_deg:.3f} deg")
    print(f"EKF joint position tracking RMSE vs encoders: {joint_pos_rmse:.5f} rad")
    print(f"EKF joint velocity tracking RMSE vs encoders: {joint_vel_rmse:.4f} rad/s")
    print("\nEKF timing:")
    print(f"Total wall time (loop): {total_wall_time:.3f} s")
    print(f"Total EKF compute time: {ekf_total_time:.3f} s")
    print(f"EKF mean step time: {ekf_mean_time * 1e3:.3f} ms")
    print(f"EKF min/max step time: {ekf_min_time * 1e3:.3f} / {ekf_max_time * 1e3:.3f} ms")
    print(f"EKF mean compute rate: {ekf_mean_rate_hz:.1f} Hz")

    pos_f, vel_f, quat_f, bg_f, joint_pos_f, joint_vel_f = ekf.get_state()
    print(f"\nFinal EKF base position: {pos_f}")
    print(f"Final EKF base velocity: {vel_f}")
    print(f"Final EKF quat [wxyz]: {quat_f}")
    print(f"Estimated gyro bias [rad/s]: {bg_f}")
    print(f"Final EKF joint positions [rad]: {joint_pos_f}")
    print(f"Final EKF joint velocities [rad/s]: {joint_vel_f}")
    
    # Convert lists to arrays for plotting
    time_vec = np.array(time_vec)
    pos_meas = np.array(pos_meas_list)
    pos_filt = np.array(pos_filt_list)
    vel_filt = np.array(vel_filt_list)
    vel_meas = np.array(vel_meas_list)
    yaw_meas = np.array(yaw_meas_list)
    yaw_filt = np.array(yaw_filt_list)
    joint_pos_meas = np.array(joint_pos_meas_list)
    joint_pos_filt = np.array(joint_pos_filt_list)
    joint_vel_meas = np.array(joint_vel_meas_list)
    joint_vel_filt = np.array(joint_vel_filt_list)
    gyro_bias = np.array(gyro_bias_list)
    quat_filt_list = np.array(quat_filt_list)
    
    # Create plots
    print("\nGenerating interactive plots...")
    
    # Figure 1: Base Position (3D)
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig1.suptitle('Base Position Tracking', fontsize=14, fontweight='bold')
    labels = ['X', 'Y', 'Z']
    for i, (ax, label) in enumerate(zip(axes1, labels)):
        ax.plot(time_vec, pos_meas[:, i], 'b-', alpha=0.6, label='OptiTrack', linewidth=1)
        ax.plot(time_vec, pos_filt[:, i], 'r-', label='EKF Filtered', linewidth=1.5)
        ax.set_ylabel(f'{label} Position [m]')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    axes1[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    
    # Figure 2: Base Velocity
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig2.suptitle('Base Velocity (EKF Estimate)', fontsize=14, fontweight='bold')
    labels = ['Vx', 'Vy', 'Vz']
    for i, (ax, label) in enumerate(zip(axes2, labels)):
        ax.plot(time_vec, vel_meas[:, i], 'g-', label='EKF Measured', linewidth=1.5)
        ax.plot(time_vec, vel_filt[:, i], 'r-', label='EKF Filtered', linewidth=1.5)
        ax.set_ylabel(f'{label} [m/s]')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    axes2[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    
    # Figure 3: Angular Velocity (from gyro measurements)
    # Compute measured (raw gyro, no bias) and filtered (bias-corrected) angular velocity in world frame
    from scipy.spatial.transform import Rotation as R
    ang_vel_meas_world_list = []
    ang_vel_filt_world_list = []
    
    for k in range(len(time_vec)):
        # Get raw gyro in body frame (measured, no bias correction)
        raw_gyro_body = gyro_velocity[k]  # shape (3,)
        
        # Get OptiTrack quaternion (for measured world frame rotation)
        q_opti = normalize_quat_wxyz(opti_rotation[k])
        rot_opti = R.from_quat([q_opti[1], q_opti[2], q_opti[3], q_opti[0]]).as_matrix()
        
        # Measured angular velocity: rotate raw gyro to world frame (no bias)
        ang_vel_meas_world = rot_opti @ raw_gyro_body
        ang_vel_meas_world_list.append(ang_vel_meas_world)
        
        # Get EKF filtered quaternion and gyro bias (from converted numpy arrays)
        q_filt = normalize_quat_wxyz(quat_filt_list[k])
        bias_filt = gyro_bias[k]
        rot_filt = R.from_quat([q_filt[1], q_filt[2], q_filt[3], q_filt[0]]).as_matrix()
        
        # Filtered angular velocity: rotate bias-corrected gyro to world frame
        ang_vel_filt_world = rot_filt @ (raw_gyro_body - bias_filt)
        ang_vel_filt_world_list.append(ang_vel_filt_world)
    
    ang_vel_meas_world = np.array(ang_vel_meas_world_list)
    ang_vel_filt_world = np.array(ang_vel_filt_world_list)
    
    fig3, axes3 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig3.suptitle('Angular Velocity Tracking', fontsize=14, fontweight='bold')
    labels = ['Wx', 'Wy', 'Wz']
    for i, (ax, label) in enumerate(zip(axes3, labels)):
        ax.plot(time_vec, np.rad2deg(ang_vel_meas_world[:, i]), 'b-', alpha=0.5, label='Raw Gyro (world)', linewidth=1)
        ax.plot(time_vec, np.rad2deg(ang_vel_filt_world[:, i]), 'r-', alpha=0.7, label='Bias-Corrected (world)', linewidth=1.5)
        ax.set_ylabel(f'{label} [deg/s]')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    axes3[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    
    # Figure 4: Yaw Tracking
    fig4, ax4 = plt.subplots(1, 1, figsize=(12, 5))
    ax4.plot(time_vec, np.rad2deg(yaw_meas), 'b-', alpha=0.6, label='OptiTrack', linewidth=1)
    ax4.plot(time_vec, np.rad2deg(yaw_filt), 'r-', label='EKF Filtered', linewidth=1.5)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Yaw [deg]')
    ax4.set_title('Base Yaw Angle Tracking', fontsize=14, fontweight='bold')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Figure 5: Gyro Bias Estimate
    fig5, axes5 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig5.suptitle('Estimated IMU Gyro Bias', fontsize=14, fontweight='bold')
    labels = ['Roll rate bias', 'Pitch rate bias', 'Yaw rate bias']
    for i, (ax, label) in enumerate(zip(axes5, labels)):
        ax.plot(time_vec, gyro_bias[:, i] * 1000, 'purple', linewidth=1.5)
        ax.set_ylabel(f'{label} [mrad/s]')
        ax.grid(True, alpha=0.3)
    axes5[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    
    # Figure 6: Joint Positions (select 4 joints for clarity)
    fig6, axes6 = plt.subplots(2, 2, figsize=(14, 10), sharex=True)
    fig6.suptitle('Joint Position Tracking (Selected Joints)', fontsize=14, fontweight='bold')
    joint_names = ['FR_hip', 'FR_knee', 'FL_hip', 'FL_knee']
    joint_indices = [0, 2, 3, 5]  # Representative joints
    axes6_flat = axes6.flatten()
    for idx, (ax, joint_idx, joint_name) in enumerate(zip(axes6_flat, joint_indices, joint_names)):
        ax.plot(time_vec, np.rad2deg(joint_pos_meas[:, joint_idx]), 'b-', alpha=0.5, label='Encoder', linewidth=1)
        ax.plot(time_vec, np.rad2deg(joint_pos_filt[:, joint_idx]), 'r-', label='EKF Filtered', linewidth=1.5)
        ax.set_ylabel(f'{joint_name} [deg]')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        if idx >= 2:
            ax.set_xlabel('Time [s]')
    plt.tight_layout()
    
    # Figure 7: Joint Velocities (select 4 joints)
    fig7, axes7_vel = plt.subplots(2, 2, figsize=(14, 10), sharex=True)
    fig7.suptitle('Joint Velocity Tracking (Selected Joints)', fontsize=14, fontweight='bold')
    axes7_vel_flat = axes7_vel.flatten()
    for idx, (ax, joint_idx, joint_name) in enumerate(zip(axes7_vel_flat, joint_indices, joint_names)):
        ax.plot(time_vec, np.rad2deg(joint_vel_meas[:, joint_idx]), 'b-', alpha=0.5, label='Encoder', linewidth=1)
        ax.plot(time_vec, np.rad2deg(joint_vel_filt[:, joint_idx]), 'r-', label='EKF Filtered', linewidth=1.5)
        ax.set_ylabel(f'{joint_name} [deg/s]')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        if idx >= 2:
            ax.set_xlabel('Time [s]')
    plt.tight_layout()
    
    # Figure 8: Tracking Errors Over Time
    fig8, axes8 = plt.subplots(2, 2, figsize=(14, 10))
    fig8.suptitle('EKF Tracking Errors', fontsize=14, fontweight='bold')
    
    # Position error
    pos_err = np.linalg.norm(pos_filt - pos_meas, axis=1)
    axes8[0, 0].plot(time_vec, pos_err * 1000, 'b-', linewidth=1)
    axes8[0, 0].set_ylabel('Position Error [mm]')
    axes8[0, 0].set_title(f'Base Position Error (RMSE={pos_tracking_rmse*1000:.2f} mm)')
    axes8[0, 0].grid(True, alpha=0.3)
    
    # Yaw error
    yaw_err = np.arctan2(np.sin(yaw_meas - yaw_filt), np.cos(yaw_meas - yaw_filt))
    axes8[0, 1].plot(time_vec, np.rad2deg(np.abs(yaw_err)), 'g-', linewidth=1)
    axes8[0, 1].set_ylabel('Yaw Error [deg]')
    axes8[0, 1].set_title(f'Base Yaw Error (RMSE={yaw_tracking_rmse_deg:.3f} deg)')
    axes8[0, 1].grid(True, alpha=0.3)
    
    # Joint position error (norm)
    joint_pos_err = np.linalg.norm(joint_pos_filt - joint_pos_meas, axis=1)
    axes8[1, 0].plot(time_vec, np.rad2deg(joint_pos_err), 'r-', linewidth=1)
    axes8[1, 0].set_ylabel('Joint Pos Error [deg]')
    axes8[1, 0].set_xlabel('Time [s]')
    axes8[1, 0].set_title(f'Joint Position Error (RMSE={np.rad2deg(joint_pos_rmse):.2f} deg)')
    axes8[1, 0].grid(True, alpha=0.3)
    
    # Joint velocity error (norm)
    joint_vel_err = np.linalg.norm(joint_vel_filt - joint_vel_meas, axis=1)
    axes8[1, 1].plot(time_vec, np.rad2deg(joint_vel_err), 'm-', linewidth=1)
    axes8[1, 1].set_ylabel('Joint Vel Error [deg/s]')
    axes8[1, 1].set_xlabel('Time [s]')
    axes8[1, 1].set_title(f'Joint Velocity Error (RMSE={np.rad2deg(joint_vel_rmse):.2f} deg/s)')
    axes8[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    print("\nDisplaying interactive plots...")
    plt.show()


if __name__ == "__main__":
    main()