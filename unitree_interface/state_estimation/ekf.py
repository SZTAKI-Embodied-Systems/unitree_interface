import numpy as np


def _normalize_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_wxyz, dtype=float).reshape(4)
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return quat / norm


def _quat_mul_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def _quat_conj_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    return np.array([w, -x, -y, -z])


def _small_angle_quat(delta_theta: np.ndarray) -> np.ndarray:
    half = 0.5 * np.asarray(delta_theta, dtype=float).reshape(3)
    norm_half = np.linalg.norm(half)
    if norm_half < 1e-12:
        return _normalize_quat_wxyz(np.array([1.0, half[0], half[1], half[2]]))

    axis = half / norm_half
    return np.array(
        [
            np.cos(norm_half),
            axis[0] * np.sin(norm_half),
            axis[1] * np.sin(norm_half),
            axis[2] * np.sin(norm_half),
        ]
    )


def _quat_to_small_angle(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = _normalize_quat_wxyz(quat_wxyz)
    if quat[0] < 0.0:
        quat = -quat
    return 2.0 * quat[1:4]


class Go2FullStateEKF:
    """
    Extended Kalman Filter for GO2 robot full state:
    - Base: position (3), velocity (3), orientation (quaternion), gyro bias (3)
    - Joints: positions (12), velocities (12)
    
    Total error-state dimension: 36
    [p_err(3), v_err(3), theta_err(3), bg_err(3), q_err(12), dq_err(12)]
    """
    
    def __init__(
        self,
        number_of_joints: int = 12,
        sigma_vel_process: float = 0.20,
        sigma_gyro_process: float = 0.50,
        sigma_gyro_bias_rw: float = 0.03,
        sigma_joint_pos_process: float = 0.001,
        sigma_joint_vel_process: float = 0.01,
        sigma_pos_meas: float = 0.01,
        sigma_vel_meas: float = 0.08,
        sigma_ori_meas_rad: float = 0.015,
        sigma_joint_pos_meas: float = 0.0005,
        sigma_joint_vel_meas: float = 0.005,
    ):
        # Base state
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.bg = np.zeros(3)
        
        # Joint state
        self.q_joints = np.zeros(number_of_joints)
        self.dq_joints = np.zeros(number_of_joints)
        
        # Covariance: 36x36 (3+3+3+3+12+12)
        self.n_joints = number_of_joints
        self.full_state_dim = 3 + 3 + 3 + 3 + number_of_joints + number_of_joints
        self.initialized = False
        
        # Process noise parameters
        self.sigma_vel_process = float(sigma_vel_process)
        self.sigma_gyro_process = float(sigma_gyro_process)
        self.sigma_gyro_bias_rw = float(sigma_gyro_bias_rw)
        self.sigma_joint_pos_process = float(sigma_joint_pos_process)
        self.sigma_joint_vel_process = float(sigma_joint_vel_process)
        
        # Measurement noise parameters
        self.sigma_pos_meas = float(sigma_pos_meas)
        self.sigma_vel_meas = float(sigma_vel_meas)
        self.sigma_ori_meas_rad = float(sigma_ori_meas_rad)
        self.sigma_joint_pos_meas = float(sigma_joint_pos_meas)
        self.sigma_joint_vel_meas = float(sigma_joint_vel_meas)

    def initialize(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        quat_wxyz: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> None:
        self.p = np.asarray(position, dtype=float).reshape(3)
        self.v = np.asarray(velocity, dtype=float).reshape(3)
        self.q = _normalize_quat_wxyz(np.asarray(quat_wxyz, dtype=float).reshape(4))
        self.bg = np.zeros(3)
        self.q_joints = np.asarray(joint_pos, dtype=float).reshape(self.n_joints)
        self.dq_joints = np.asarray(joint_vel, dtype=float).reshape(self.n_joints)
        
        # Initialize covariance
        self.P = np.eye(self.full_state_dim) * 1e-3
        # Higher initial uncertainty for base velocity and gyro bias
        self.P[3:6, 3:6] = np.eye(3) * 1e-2
        self.P[9:12, 9:12] = np.eye(3) * 1e-2
        
        self.initialized = True

    def predict(self, gyro_body: np.ndarray, dt: float) -> None:
        if not self.initialized:
            return

        dt = float(dt)
        if dt <= 0.0:
            return

        # Base prediction
        omega = np.asarray(gyro_body, dtype=float).reshape(3) - self.bg
        self.p = self.p + self.v * dt
        dq = _small_angle_quat(omega * dt)
        self.q = _normalize_quat_wxyz(_quat_mul_wxyz(self.q, dq))
        
        # Joint prediction (constant velocity model)
        self.q_joints = self.q_joints + self.dq_joints * dt

        # Jacobian F (36x36)
        F = np.eye(self.full_state_dim)
        F[0:3, 3:6] = np.eye(3) * dt  # p depends on v
        F[6:9, 9:12] = -np.eye(3) * dt  # theta depends on bg
        F[12:12+self.n_joints, 12+self.n_joints:12+self.n_joints*2] = np.eye(self.n_joints) * dt  # q_joints depends on dq_joints

        # Process noise Q (36x36)
        Q = np.zeros((self.full_state_dim, self.full_state_dim))
        # Base noise
        Q[0:3, 0:3] = np.eye(3) * (self.sigma_vel_process**2) * (dt**2)
        Q[3:6, 3:6] = np.eye(3) * (self.sigma_vel_process**2) * dt
        Q[6:9, 6:9] = np.eye(3) * (self.sigma_gyro_process**2) * dt
        Q[9:12, 9:12] = np.eye(3) * (self.sigma_gyro_bias_rw**2) * dt
        # Joint noise
        Q[12:12+self.n_joints, 12:12+self.n_joints] = np.eye(self.n_joints) * (self.sigma_joint_pos_process**2) * dt
        Q[12+self.n_joints:12+self.n_joints*2, 12+self.n_joints:12+self.n_joints*2] = np.eye(self.n_joints) * (self.sigma_joint_vel_process**2) * dt

        self.P = F @ self.P @ F.T + Q

    def update_optitrack(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        quat_wxyz: np.ndarray,
    ) -> None:
        """Update using OptiTrack base measurements (position, velocity, orientation)"""
        if not self.initialized:
            return

        z_p = np.asarray(position, dtype=float).reshape(3)
        z_v = np.asarray(velocity, dtype=float).reshape(3)
        z_q = _normalize_quat_wxyz(np.asarray(quat_wxyz, dtype=float).reshape(4))

        # Orientation residual
        q_err = _quat_mul_wxyz(z_q, _quat_conj_wxyz(self.q))
        r_ori = _quat_to_small_angle(q_err)

        # Residual vector (9D: 3 pos + 3 vel + 3 ori)
        r = np.hstack((z_p - self.p, z_v - self.v, r_ori))

        # Measurement Jacobian H (9x36)
        H = np.zeros((9, self.full_state_dim))
        H[0:3, 0:3] = np.eye(3)  # position
        H[3:6, 3:6] = np.eye(3)  # velocity
        H[6:9, 6:9] = np.eye(3)  # orientation

        # Measurement noise R (9x9)
        Rm = np.zeros((9, 9))
        Rm[0:3, 0:3] = np.eye(3) * (self.sigma_pos_meas**2)
        Rm[3:6, 3:6] = np.eye(3) * (self.sigma_vel_meas**2)
        Rm[6:9, 6:9] = np.eye(3) * (self.sigma_ori_meas_rad**2)

        # Kalman update
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r

        # State correction
        self.p = self.p + dx[0:3]
        self.v = self.v + dx[3:6]
        dq_corr = _small_angle_quat(dx[6:9])
        self.q = _normalize_quat_wxyz(_quat_mul_wxyz(dq_corr, self.q))
        self.bg = self.bg + dx[9:12]
        self.q_joints = self.q_joints + dx[12:12+self.n_joints]
        self.dq_joints = self.dq_joints + dx[12+self.n_joints:12+self.n_joints*2]

        # Covariance update
        I = np.eye(self.full_state_dim)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ Rm @ K.T

    def update_joints(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> None:
        """Update using joint encoder measurements"""
        if not self.initialized:
            return

        z_q = np.asarray(joint_pos, dtype=float).reshape(self.n_joints)
        z_dq = np.asarray(joint_vel, dtype=float).reshape(self.n_joints)

        # Residual vector (24D: 12 pos + 12 vel)
        r = np.hstack((z_q - self.q_joints, z_dq - self.dq_joints))

        # Measurement Jacobian H (24x36)
        H = np.zeros((2*self.n_joints, self.full_state_dim))
        H[0:self.n_joints, 12:12+self.n_joints] = np.eye(self.n_joints)  # joint positions
        H[self.n_joints:2*self.n_joints, 12+self.n_joints:self.full_state_dim] = np.eye(self.n_joints)  # joint velocities

        # Measurement noise R (24x24)
        Rm = np.zeros((2*self.n_joints, 2*self.n_joints))
        Rm[0:self.n_joints, 0:self.n_joints] = np.eye(self.n_joints) * (self.sigma_joint_pos_meas**2)
        Rm[self.n_joints:2*self.n_joints, self.n_joints:2*self.n_joints] = np.eye(self.n_joints) * (self.sigma_joint_vel_meas**2)

        # Kalman update
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r

        # State correction
        self.p = self.p + dx[0:3]
        self.v = self.v + dx[3:6]
        dq_corr = _small_angle_quat(dx[6:9])
        self.q = _normalize_quat_wxyz(_quat_mul_wxyz(dq_corr, self.q))
        self.bg = self.bg + dx[9:12]
        self.q_joints = self.q_joints + dx[12:12+self.n_joints]
        self.dq_joints = self.dq_joints + dx[12+self.n_joints:12+self.n_joints*2]

        # Covariance update
        I = np.eye(self.full_state_dim)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ Rm @ K.T

    def get_state(self):
        """Returns: (base_pos, base_vel, base_quat_wxyz, gyro_bias, joint_pos, joint_vel)"""
        return (
            self.p.copy(),
            self.v.copy(),
            self.q.copy(),
            self.bg.copy(),
            self.q_joints.copy(),
            self.dq_joints.copy(),
        )
