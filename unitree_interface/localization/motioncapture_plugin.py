import struct
import time
from multiprocessing import shared_memory
import threading

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter
import motioncapture

from unitree_interface.localization.base_plugin import BaseLocalizationPlugin

class OptiTrackDemo:
    def __init__(self, optitrack_hostname, optitrack_object_name, optitrack_z_offset):
        # OptiTrack hostname and object name
        self.optitrack_hostname = optitrack_hostname
        self.optitrack_object_name = optitrack_object_name
        self.optitrack_z_offset = optitrack_z_offset
        
        # Connect to OptiTrack
        print("Connecting to OptiTrack at", self.optitrack_hostname)
        self.mocap = motioncapture.connect(
            "optitrack",
            {'hostname': self.optitrack_hostname}
        )
        print("Connected to OptiTrack at", self.optitrack_hostname)

        # Initialize previous values for velocity computation
        self.prev_time = None
        self.prev_position = None
        self.prev_quaternion = None

        # Low-pass filter parameters
        self.cutoff_freq = 5.0  # Cut-off frequency of the filter (Hz)
        self.filter_order = 2
        self.fs = 100.0  # Sampling frequency (Hz)
        self.b, self.a = butter(
            self.filter_order, self.cutoff_freq / (0.5 * self.fs), btype="low"
        )

        # Initialize data buffers for filtering
        self.vel_buffer = []
        self.omega_buffer = []

        # Initialize shared memory
        self.shared_mem_name = "mocap_state_shm"
        self.shared_mem_size = 8 + 13 * 8  # 8 bytes for utime (int64), 13 float64s (13*8 bytes)
        try:
            self.state_shm = shared_memory.SharedMemory(name=self.shared_mem_name, create=True, size=self.shared_mem_size)
        except FileExistsError:
            # Shared memory already exists, attach to it
            self.state_shm = shared_memory.SharedMemory(name=self.shared_mem_name, create=False)
        self.state_buffer = self.state_shm.buf

    def get_optitrack_data(self):
        try:
            self.mocap.waitForNextFrame()
            body = self.mocap.rigidBodies.get(self.optitrack_object_name)
            
            if body is None:
                print(f"Cannot get the pose of `{self.optitrack_object_name}`.")
                return None, None, None

            current_time = time.time()

            # Position and orientation
            position = np.array([body.position[0], body.position[1], body.position[2]])
            position[2] = position[2] + self.optitrack_z_offset
            
            # OptiTrack returns quaternion as (w, x, y, z)
            quaternion = np.array([body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w])

            return current_time, position, quaternion
        except Exception as e:
            print(f"Error retrieving OptiTrack data: {e}")
            return None, None, None

    def compute_velocities(self, current_time, position, quaternion):
        # Initialize velocities
        linear_velocity = np.zeros(3)
        angular_velocity = np.zeros(3)

        if (
            self.prev_time is not None
            and self.prev_position is not None
            and self.prev_quaternion is not None
        ):
            dt = current_time - self.prev_time
            if dt > 0:
                # Linear velocity
                dp = position - self.prev_position
                linear_velocity = dp / dt

                # Angular velocity
                prev_rot = R.from_quat(self.prev_quaternion)
                curr_rot = R.from_quat(quaternion)
                delta_rot = curr_rot * prev_rot.inv()
                delta_angle = delta_rot.as_rotvec()
                angular_velocity = delta_angle / dt
        else:
            # First data point; velocities remain zero
            pass

        # Update previous values
        self.prev_time = current_time
        self.prev_position = position
        self.prev_quaternion = quaternion

        return linear_velocity, angular_velocity

    def low_pass_filter(self, data_buffer, new_data):
        # Append new data to the buffer
        data_buffer.append(new_data)
        # Keep only the last N samples (buffer size)
        buffer_size = int(self.fs / self.cutoff_freq) * 3
        if len(data_buffer) > buffer_size:
            data_buffer.pop(0)
        # Apply low-pass filter if enough data points are available
        if len(data_buffer) >= self.filter_order + 1:
            data_array = np.array(data_buffer)
            filtered_data = lfilter(self.b, self.a, data_array, axis=0)[-1]
            return filtered_data
        else:
            return new_data  # Not enough data to filter; return the new data as is

    def main_loop(self):
        print("Starting OptiTrack data acquisition")
        try:
            while True:
                # Get OptiTrack data
                current_time, position, quaternion = self.get_optitrack_data()
                if position is None:
                    time.sleep(0.01)         
                    continue

                #print(f"OptiTrack Time: {current_time}, Position: {position}, Quaternion: {quaternion}")

                # Compute velocities
                linear_velocity, angular_velocity = self.compute_velocities(
                    current_time, position, quaternion
                )

                # Apply low-pass filter to velocities
                filtered_linear_velocity = self.low_pass_filter(
                    self.vel_buffer, linear_velocity
                )
                filtered_angular_velocity = self.low_pass_filter(
                    self.omega_buffer, angular_velocity
                )

                # Prepare data to pack
                utime = int(current_time * 1e6)  # int64
                data_to_pack = [utime]
                data_to_pack.extend(position.tolist())
                data_to_pack.extend(quaternion.tolist())
                data_to_pack.extend(filtered_linear_velocity.tolist())
                data_to_pack.extend(filtered_angular_velocity.tolist())

                # Pack data into shared memory buffer
                struct_format = "q13d"
                struct.pack_into(struct_format, self.state_buffer, 0, *data_to_pack)

                # Sleep to mimic sampling rate
                time.sleep(1.0 / self.fs)

        except KeyboardInterrupt:
            print("Exiting OptiTrack data acquisition.")
        finally:
            # Close and unlink shared memory
            try:
                self.state_shm.close()
                self.state_shm.unlink()
            except:
                pass


class OptiTrackPlugin(BaseLocalizationPlugin):
    def __init__(self, config):
        self.time = time.time()
        # Initialize OptiTrack thread
        optitrack_demo = OptiTrackDemo(
            config['optitrack_hostname'], 
            config['optitrack_object_name'], 
            config.get('optitrack_z_offset', 0.0)
        )
        self.optitrack_thread = threading.Thread(target=optitrack_demo.main_loop)
        self.optitrack_thread.start()

        # Initialize shared memory
        self.shared_mem_name = "mocap_state_shm"
        self.shared_mem_size = 8 + 13 * 8  # 8 bytes for utime (int64), 13 float64s (13*8 bytes)
        self.mocap_shm = shared_memory.SharedMemory(name=self.shared_mem_name, create=False, size=self.shared_mem_size)
        self.state_buffer = self.mocap_shm.buf

    def get_state(self):
        # Unpack data from shared memory
        struct_format = "q13d"
        data = struct.unpack_from(struct_format, self.state_buffer, 0)

        # Extract position, quaternion, linear velocity, and angular velocity
        utime = data[0]
        position = np.array(data[1:4])
        quaternion = np.array(data[4:8])
        quaternion = np.roll(quaternion, 1)  # change quaternion from xyzw to wxyz
        linear_velocity = np.array(data[8:11])
        angular_velocity = np.array(data[11:14])

        # Combine position and quaternion into qpos
        qpos = np.concatenate([position, quaternion])
        # Combine linear and angular velocities into qvel
        qvel = np.concatenate([linear_velocity, angular_velocity])
        self.time = utime
        return np.concatenate([qpos, qvel])

    def get_last_update_time(self):
        return self.time
    
