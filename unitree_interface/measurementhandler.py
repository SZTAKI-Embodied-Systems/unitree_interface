import time
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R

from unitree_interface.unitreeinterface import UnitreeInterface, UnitreeInterfaceGO2, UnitreeInterfaceG1
from unitree_interface.state_estimation.ekf import EKF
from unitree_interface.localization import load_plugin

class MeasurementHandler:
    def __init__(self, robot_interface: UnitreeInterface,
                 localization_plugin_config: dict = None, 
                 use_ekf: bool = True,
                 model_nq: int = None,
                 model_nv: int = None,
                 model_nu: int = None):
        self.robot_interface = robot_interface
        self.plugin_config = localization_plugin_config

        self.use_ekf = use_ekf
        self.ekf = EKF() if use_ekf else None
        self.last_ekf_time = None

        self.model_nq = model_nq
        self.model_nv = model_nv
        self.model_nu = model_nu

        self.localization_plugin = load_plugin(self.plugin_config["localization_plugin"])
        self.localization_plugin = self.localization_plugin(self.plugin_config)
        self.localization_timeout_sec = self.plugin_config["localization_timeout_sec"]
        self.moving_average_window = int(self.plugin_config.get("measurement_moving_average_window", 5))
        if self.moving_average_window < 1:
            self.moving_average_window = 1

        self._ma_buffers = {
            "opti_pos": deque(maxlen=self.moving_average_window),
            "opti_vel": deque(maxlen=self.moving_average_window),
            "opti_quat": deque(maxlen=self.moving_average_window),
            "imu_gyro": deque(maxlen=self.moving_average_window),
            "joint_pos": deque(maxlen=self.moving_average_window),
            "joint_vel": deque(maxlen=self.moving_average_window),
            "joint_tau": deque(maxlen=self.moving_average_window),
        }

        self.raw_state_q = np.zeros(self.model_nq)
        self.raw_state_dq = np.zeros(self.model_nv)
        self.raw_control_u = np.zeros(self.model_nu)

        self.filtered_state_q = np.zeros(self.model_nq)
        self.filtered_state_dq = np.zeros(self.model_nv)

    def _moving_average(self, key, value):
        arr = np.asarray(value, dtype=float)
        self._ma_buffers[key].append(arr.copy())
        return np.mean(np.asarray(self._ma_buffers[key]), axis=0)

    def process_measurements(self):
        localization_output = self.localization_plugin.get_state()
        if localization_output is None:
            return
        now = time.time()
        localization_time = self.localization_plugin.get_last_update_time()
        if now - localization_time > self.localization_timeout_sec:
            print(f"[WARNING] Localization plugin timeout: {now - localization_time} s")
            return
        
        # OptiTrack measurements
        opti_base_pos = np.asarray(localization_output[:3], dtype=float)
        opti_base_vel = np.asarray(localization_output[7:10], dtype=float)
        opti_base_quat_wxyz = np.asarray(localization_output[3:7], dtype=float)
        opti_base_quat_wxyz = opti_base_quat_wxyz / np.linalg.norm(opti_base_quat_wxyz)

        robot_base_ang_vel = self.robot_interface.getLowStateImuGyroscope()
        robot_joint_pos = self.robot_interface.getLowStateJointPos()
        robot_joint_vel = self.robot_interface.getLowStateJointVel()
        robot_joint_tau = self.robot_interface.getLowStateTauEst()

        if self.use_ekf:
            # Only smooth measurements when feeding the EKF.
            ekf_opti_base_pos = self._moving_average("opti_pos", opti_base_pos)
            ekf_opti_base_vel = self._moving_average("opti_vel", opti_base_vel)
            ekf_opti_base_quat_wxyz = self._moving_average("opti_quat", opti_base_quat_wxyz)
            quat_norm = np.linalg.norm(ekf_opti_base_quat_wxyz)
            if quat_norm > 1e-8:
                ekf_opti_base_quat_wxyz = ekf_opti_base_quat_wxyz / quat_norm
            else:
                ekf_opti_base_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])

            ekf_robot_base_ang_vel = self._moving_average("imu_gyro", robot_base_ang_vel)
            ekf_robot_joint_pos = self._moving_average("joint_pos", robot_joint_pos)
            ekf_robot_joint_vel = self._moving_average("joint_vel", robot_joint_vel)

            if self.last_ekf_time is None:
                self.ekf.initialize(
                    position=ekf_opti_base_pos,
                    velocity=ekf_opti_base_vel,
                    quat_wxyz=ekf_opti_base_quat_wxyz,
                    joint_pos=ekf_robot_joint_pos,
                    joint_vel=ekf_robot_joint_vel,
                )
                self.last_ekf_time = now
            else:
                dt = max(1e-4, min(0.05, now - self.last_ekf_time))
                self.ekf.predict(gyro_body=ekf_robot_base_ang_vel, dt=dt)
                self.ekf.update_optitrack(position=ekf_opti_base_pos, velocity=ekf_opti_base_vel, quat_wxyz=ekf_opti_base_quat_wxyz)
                self.ekf.update_joints(joint_pos=ekf_robot_joint_pos, joint_vel=ekf_robot_joint_vel)
                self.last_ekf_time = now

            filt_base_pos, filt_base_vel, filt_base_quat_wxyz, gyro_bias, filt_joint_pos, filt_joint_vel = self.ekf.get_state()
        else:
            filt_base_pos = opti_base_pos
            filt_base_vel = opti_base_vel
            filt_base_quat_wxyz = opti_base_quat_wxyz
            gyro_bias = np.zeros(3)
            filt_joint_pos = robot_joint_pos
            filt_joint_vel = robot_joint_vel

        rot = R.from_quat([opti_base_quat_wxyz[1], opti_base_quat_wxyz[2], opti_base_quat_wxyz[3], opti_base_quat_wxyz[0]]).as_matrix()
        raw_ang_vel_world = rot @ (robot_base_ang_vel - gyro_bias)
        self.raw_state_q = np.concatenate([opti_base_pos, opti_base_quat_wxyz, robot_joint_pos])
        self.raw_state_dq = np.concatenate([opti_base_vel, raw_ang_vel_world, robot_joint_vel])
        self.raw_control_u = np.array(robot_joint_tau)

        rot = R.from_quat([filt_base_quat_wxyz[1], filt_base_quat_wxyz[2], filt_base_quat_wxyz[3], filt_base_quat_wxyz[0]]).as_matrix()
        filt_ang_vel_world = rot @ (robot_base_ang_vel - gyro_bias)
        self.filtered_state_q = np.concatenate([filt_base_pos, filt_base_quat_wxyz, filt_joint_pos])
        self.filtered_state_dq = np.concatenate([filt_base_vel, filt_ang_vel_world, filt_joint_vel])

    def get_raw_state_q(self):
        return self.raw_state_q

    def get_raw_state_dq(self):
        return self.raw_state_dq

    def get_raw_state_u(self):
        return self.raw_control_u

    def get_filtered_state_q(self):
        return self.filtered_state_q

    def get_filtered_state_dq(self):
        return self.filtered_state_dq