import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Union, List, Optional, Callable, Literal
import threading
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_, unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_

from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowState_GO2
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmd_GO2
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmd_G1
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowState_G1

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import Thread, RecurrentThread

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient as SportClientGO2
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient as LocoClientG1

from unitree_interface.motormodel import Go2MotorModel, G1MotorModel

class InterpTargets:
    def __init__(self, num_joints: int):
        self.prev_q = [0.0]*num_joints
        self.prev_dq = [0.0]*num_joints
        self.prev_tau = [0.0]*num_joints

        self.next_q = [0.0]*num_joints
        self.next_dq = [0.0]*num_joints
        self.next_tau = [0.0]*num_joints

    def set_prev_targets(self, new_q, new_dq, new_tau):
        self.prev_q = new_q
        self.prev_dq = new_dq
        self.prev_tau = new_tau

    def set_next_targets(self, new_q, new_dq, new_tau):
        self.next_q = new_q
        self.next_dq = new_dq
        self.next_tau = new_tau

    def get_prev_targets(self):
        return self.prev_q, self.prev_dq, self.prev_tau
    
    def get_next_targets(self):
        return self.next_q, self.next_dq, self.next_tau
    
    def shift_targets(self):
        self.prev_q = self.next_q
        self.prev_dq = self.next_dq
        self.prev_tau = self.next_tau



class UnitreeInterface(ABC):
    def __init__(self,
                 network_interface: str,
                 low_state_callback: Optional[Callable] = None,
                 low_cmd_pub_dt: float = 0.002):
        
        self.network_interface = network_interface
        self.low_state_callback = low_state_callback
        self.low_cmd_pub_dt = low_cmd_pub_dt
        self.low_cmd_lock = threading.Lock()

        self.emergency_event = threading.Event()
        
        self.connected = False

        self.use_interp = threading.Event()
        self.going_init_pos = threading.Event()
        self.going_stopping_pos = threading.Event()
            
        self.interp_progress = 0.0
        self.interp_total_step = 10
        self.interp_targets = None # Initialized in child class after total_joint_num is known

    @abstractmethod
    def Init(self):
        ''' Initialize the Unitree interface components '''
        pass

    @abstractmethod
    def _initLowCmd(self):
        ''' Initialize the low-level command message with default values '''
        pass

    @abstractmethod
    def LowCmdMotorUpdateControl(self,
                                 q: List[float], 
                                 dq: Union[float, List[float]], 
                                 tau: Union[float, List[float]],
                                 kp: Union[float, List[float]] = None, 
                                 kd: Union[float, List[float]] = None
                                 ):
        ''' Update low-level motor command values'''
        pass

    def _initLowPub(self, msg_type):
        self.low_pub = ChannelPublisher("rt/lowcmd", msg_type)
        self.low_pub.Init()
        
    def _initLowSub(self, msg_type):
        self.low_sub = ChannelSubscriber("rt/lowstate", msg_type)
        self.low_sub.Init(self._internalLowStateCallback, 1) # callback function
    
    def _internalLowStateCallback(self, msg: LowState_GO2):
        """Internal callback that executes class logic before user callback"""
        self.low_state_msg = msg

        # Check connection status on first message
        if not self.connected:
            self.connected = True

        # Call the external low state message handler
        if self.low_state_callback is not None:
            self.low_state_callback()
        else:
            print("[Interface] No low state callback function provided.")

    def _initMotionSwitcherClient(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        print("[Interface] Robot MSC checkmode status: ", status, result)
        while result['name']:
            self.msc.ReleaseMode()
            print("[Interface] Robot MSC checkmode status: ", status, result)
            status, result = self.msc.CheckMode()
            time.sleep(1)

    def _startLowCmdWriteThread(self):
        self.low_cmd_write_thread = RecurrentThread(
            interval=self.low_cmd_pub_dt,
            target=self._lowCmdWrite,
            name="lowCmdWriteThread",
        )
        while not self.connected:
            print("[Interface] Waiting for connection to robot before starting low-level command thread.")
            time.sleep(1.0)

        if self.connected:
            print("[Interface] Starting low-level command write thread.")
            self.low_cmd_write_thread.Start()

    def _stopLowCmdWriteThread(self):
        if self.low_cmd_write_thread is not None:
            print("[Interface] Stopping low-level command write thread.")
            self.low_cmd_write_thread.Wait(timeout=1.0)

    @abstractmethod
    def _lowCmdWrite(self):
        ''' Publish low-level motor commands to the robot '''
        pass

    def CheckConnectionStatus(self):
        ''' Check if the robot is connected by waiting for low state messages \n
         You must set robot.connected = True in the low state callback function for this function to work properly'''
        self.connected = False

        start_wait = time.time()
        while time.time() - start_wait < 3:
            if self.connected:
                print("[Interface] Successfully connected to robot")
                break
        if not self.connected:
            print("[WARNING] No robot state received within timeout, check network interface settings")

    def _clampVariable(self, x, xmin, xmax):
        return max(xmin, min(x, xmax))

    def CloseConnection(self):
        print("[Interface] Closing Unitree interface connection.")
        pass

    def _printLowCmd(self):
        for i in range(self.total_joint_num):
            print(f"{i} mode: {self.low_cmd.mode_machine} q:{self.low_cmd.motor_cmd[i].q:.3f} dq:{self.low_cmd.motor_cmd[i].dq:.3f} ", sep="")
            print(f"tau:{self.low_cmd.motor_cmd[i].tau:.3f} kp:{self.low_cmd.motor_cmd[i].kp} kd:{self.low_cmd.motor_cmd[i].kd}", sep="")
        #print([self.low_cmd.motor_cmd[i].q for i in range(self.total_joint_num)]) # DEBUG


@dataclass
class UnitreeInterfaceDataGO2:
    HIGHLEVEL = 0xEE
    LOWLEVEL = 0xFF
    TRIGERLEVEL = 0xF0
    PosStopF = 2.146e9
    VelStopF = 16000.0
    pos_lying = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65,]
    q_standing = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]

class UnitreeInterfaceGO2(UnitreeInterface):
    def __init__(self,
                 network_interface: str,
                 low_state_callback = None,
                 low_cmd_pub_dt: float = 0.002):
        super().__init__(network_interface, low_state_callback, low_cmd_pub_dt)
        
        self.total_joint_num = 12
        self.kp_default = [60.0] * self.total_joint_num
        self.kd_default = [5.0] * self.total_joint_num
        self.motor_model = Go2MotorModel()
        self.interp_targets = InterpTargets(self.total_joint_num)

    def Init(self):
        ''' Initialize the Unitree Go2 interface components 
        The function sets up the LowCmd, LowSub, LowPub, SportClient, and MotionSwitcherClient components.'''

        self.crc = CRC()
        self.data = UnitreeInterfaceDataGO2()
        ChannelFactoryInitialize(0, self.network_interface)
        
        self._initLowCmd()              # Create low cmd message
        self._initLowSub(LowState_GO2)  # Initialize low state subscriber
        self._initLowPub(LowCmd_GO2)    # Initialize low cmd publisher
        
        self.low_state = unitree_go_msg_dds__LowState_()
        self.low_cmd_write_thread = None

        try:
            self._initSportClient()
        except Exception as e:
            raise RuntimeError(f"[Interface] Failed to initialize SportClient: {e}")
        try:
            self._initMotionSwitcherClient()
        except Exception as e:
            raise RuntimeError(f"[Interface] Failed to initialize MotionSwitcherClient: {e}")
    def _initLowCmd(self):
        ''' Initialize the low-level command message with default values
            (stop motors and keep position) '''
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= self.data.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = self.data.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0
        
    def _initSportClient(self):
        self.sc = SportClientGO2()
        self.sc.SetTimeout(5.0)
        self.sc.Init()

    def LowCmdMotorUpdateControl(self,
                                  q: List[float], 
                                  dq: Union[float, List[float]], 
                                  tau: Union[float, List[float]],
                                  kp: Union[float, List[float]] = None, 
                                  kd: Union[float, List[float]] = None
                                  ):
        ''' Update low-level motor command values'''
        # Convert inputs to lists if they are scalars
        if not (isinstance(q, list) or isinstance(q, np.ndarray)):
            print(f"[Interface] LowCmdMotorUpdateControl: q must be a list or numpy array")
            return
        if isinstance(dq, (int, float)): dq = [dq] * self.total_joint_num
        if isinstance(tau, (int, float)): tau = [tau] * self.total_joint_num
        
        if kp is None:
            kp = self.kp_default
        elif isinstance(kp, (int, float)):
            kp = [kp] * self.total_joint_num
        if kd is None:
            kd = self.kd_default
        elif isinstance(kd, (int, float)):
            kd = [kd] * self.total_joint_num

        with self.low_cmd_lock:
            for i in range(self.total_joint_num):
                self.low_cmd.motor_cmd[i].q = self._clampVariable(q[i], self.motor_model.min_position_list[i], self.motor_model.max_position_list[i])
                self.low_cmd.motor_cmd[i].dq = self._clampVariable(dq[i], self.motor_model.min_velocity_list[i], self.motor_model.max_velocity_list[i])
                self.low_cmd.motor_cmd[i].tau = self._clampVariable(tau[i], self.motor_model.min_torque_list[i], self.motor_model.max_torque_list[i])

                self.low_cmd.motor_cmd[i].kp = kp[i]
                self.low_cmd.motor_cmd[i].kd = kd[i]

    # TODO move to parent class
    def _lowCmdWrite(self):
        ''' Publish low-level motor commands to the robot '''
        if self.emergency_event.is_set():
            self.msc.ReleaseMode()
            print("EMERGENCY STOP activated, sent ReleaseMode command to robot.")

        if self.use_interp.is_set():
            self._lowCmdMotorUpdateInterpLoop()

        # self._printLowCmd()  # DEBUG

        with self.low_cmd_lock:
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_pub.Write(self.low_cmd)


    def _lowCmdMotorUpdateInterpLoop(self):
        """ Interpolation loop for low-level motor command updates """
        prev_targets = self.interp_targets.get_prev_targets()
        next_targets = self.interp_targets.get_next_targets()

        self.interp_progress += 1.0 / self.interp_total_step
        self.interp_progress = min(self.interp_progress, 1)

        q = [(1 - self.interp_progress) * prev_targets[0][i] + self.interp_progress * next_targets[0][i] for i in range(12)]
        dq = 0.0
        kp = self.kp_default  # ADD kp and kd to interp handling class !!! otherwise it cant be changed here
        kd = self.kd_default
        tau = 0.0
        self.LowCmdMotorUpdateControl(q, dq, tau, kp, kd)
        # print(f"Interp progress: {self.interp_progress:.3f}", f"{q[:3]}") # DEBUG 

        if self.interp_progress == 1.0 and self.going_init_pos.is_set():
            print("[Interface] Reached init target position.")
            self.interp_total_step = 40
            self.going_init_pos.clear()
            self.use_interp.clear()

        if self.interp_progress == 1.0 and self.going_stopping_pos.is_set():
            print("[Interface] Reached stop target position.")
            self.interp_total_step = 10
            self.use_interp.clear()
            self._stopLowCmdWriteThread()
            # self.going_stopping_pos.clear()


    def LowCmdMotorUpdateInterpTarget(self, tar_q=None, tar_dq=None, tar_tau=None, kp=None, kd=None, set_cur_pos_as_target=False):
        ''' Update low-level motor command values for target position control'''

        if not self.going_init_pos.is_set() and not self.going_stopping_pos.is_set():
            if set_cur_pos_as_target: # Initialize interpolation targets
                current_q = [ms.q for ms in self.low_state_msg.motor_state]
                self.interp_targets.set_prev_targets(current_q, [0] * self.total_joint_num, [0] * self.total_joint_num) # if first run set current pos as prev pos
                if not tar_q:
                    self.interp_targets.set_next_targets(current_q, [0] * self.total_joint_num, [0] * self.total_joint_num) # if no target pos given set current pos as target pos
                else: 
                    self.interp_targets.set_next_targets(tar_q, [0]*self.total_joint_num, [0]*self.total_joint_num)
                self.interp_progress = 0.0
                
            elif tar_q != None and tar_dq != None and tar_tau != None: # Set new target pos
                self.interp_targets.shift_targets()
                self.interp_targets.set_next_targets(tar_q, tar_dq, tar_tau)
                self.interp_progress = 0.0

 
    def StartLowCmdControl(self, keep_still:bool = False,
                           target_pose:str = None,
                           target_q: list = None):
        ''' Start the low-level command control thread while keeping the robot in current position or interpolating to target position '''
        if keep_still:
            current_q = [ms.q for ms in self.low_state_msg.motor_state]
            
            self.LowCmdMotorUpdateControl(current_q, 0.0, 0.0)
            print('[Interface] Starting low-level command control, keeping current position.')
        else:
            if target_q is None:
                target_q = self.data.q_standing
            self.interp_total_step = 600
            self.LowCmdMotorUpdateInterpTarget(tar_q=target_q, set_cur_pos_as_target=True)
            self.use_interp.set()
            self.going_init_pos.set()
            print('[Interface] Starting low-level command control, interpolating to target position.')
        
        self._startLowCmdWriteThread() # Start publishing low-level commands after the initial target posion is set
        # RESET INTERP TOTAL STEP TO DEFAULT

    def StopLowCmdControl(self, stop_target_q: list = None):
        print("[Interface] Stopping low-level command control, interpolating to stop position.")
        if stop_target_q is None:
            stop_target_q = self.data.pos_lying
        self.interp_total_step = 600
        self.LowCmdMotorUpdateInterpTarget(tar_q=stop_target_q, set_cur_pos_as_target=True)
        self.use_interp.set()
        self.going_stopping_pos.set()
        print('[Interface] Starting stopping process, interpolating to stop position.')

    def CloseConnection(self):
        super().CloseConnection()
        self.StopLowCmdControl()
        # self._stopLowCmdMotorPublishThread() # Graceful shutdown not working yet


@dataclass
class UnitreeInterfaceDataG1:
    """ PR Mode: Controls the Pitch (P) and Roll (R) motors of the ankle joint (default mode, corresponding to the URDF model).
        AB Mode: Directly controls the A and B motors of the ankle joint (requires users to calculate the parallel mechanism kinematics themselves). """
    MODE_PR = 0 # Series Control for Pitch/Roll Joints (default)
    MODE_AB = 1 # Parallel Control for A/B Joints
    # MODE_MACHINE: G1 Type：4：23-Dof;5:29-Dof;6:27-Dof(29Dof Fitted at the waist)
    MODE_MACHINE_DOF_DICT = {4:23, 5:29, 6:27}
    # q_standing = [                    # Standing pose with slightly bent arms and legs
    #     -0.1, 0, 0, 0.3, -0.2, 0,     # left leg
    #     -0.1, 0, 0, 0.3, -0.2, 0,     # right leg
    #     0, 0, 0,                      # waist
    #     0.2,  0.2, 0, 1.28, 0, 0, 0,  # left arm
    #     0.2, -0.2, 0, 1.28, 0, 0, 0   # right arm
    # ]
    q_standing = [              # Standing pose with straight arms and legs
        0, 0, 0, 0, 0, 0,       # left leg
        0, 0, 0, 0, 0, 0,       # right leg
        0, 0, 0,                # waist
        0, 0, 0, 1.1, 0, 0, 0,  # left arm
        0, 0, 0, 1.1, 0, 0, 0   # right arm
    ]

class UnitreeInterfaceG1(UnitreeInterface):
    def __init__(self,
                 network_interface: str,
                 low_state_callback = None,
                 low_cmd_pub_dt: float = 0.002):
        super().__init__(network_interface, low_state_callback, low_cmd_pub_dt)
        
        self.total_joint_num = 29
        self.all_motor_ids = list(range(self.total_joint_num))
        self.available_motor_ids = self.all_motor_ids.copy()   # Default to all motors controlled, set by first LowState message
        self.controlled_motor_ids = self.all_motor_ids.copy()  # Default to all motors controlled, can be set by user
        # used_motor_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17, 18, 22, 23, 24, 25]

        # The waist roll and pitch are zeroed because the joint is physically locked)
        self.kp_default = [
            60, 60, 60, 100, 40, 400,     # legs
            60, 60, 60, 100, 40, 40,      # legs
            60, 0, 0,                     # waist (yaw, roll, pitch, default: (60,40,40))
            40, 40, 40, 40,  40, 40, 40,  # arms
            40, 40, 40, 40,  40, 40, 40   # arms
        ]
        self.kd_default = [
            1, 1, 1, 2, 1, 1,     # legs
            1, 1, 1, 2, 1, 1,     # legs
            1, 0, 0,              # waist (yaw, roll, pitch, default: (1,1,1))
            1, 1, 1, 1, 1, 1, 1,  # arms
            1, 1, 1, 1, 1, 1, 1   # arms
        ]
        self.motor_model = G1MotorModel()
        self.interp_targets = InterpTargets(self.total_joint_num)
        self.mode_machine = None # Machine mode (rewritten on first incoming message)

    def Init(self):
        ''' Initialize the Unitree G1 interface components 
        The function sets up the LowCmd, LowSub, LowPub, LocoClient, and MotionSwitcherClient components.'''

        self.crc = CRC()
        self.data = UnitreeInterfaceDataG1()
        ChannelFactoryInitialize(0, self.network_interface)
        
        self._initLowCmd()             # Create low cmd message
        self._initLowSub(LowState_G1)  # Initialize low state subscriber
        self._initLowPub(LowCmd_G1)    # Initialize low cmd publisher
        
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.low_cmd_write_thread = None

        try:
            self._initMotionSwitcherClient()
        except Exception as e:
            raise RuntimeError(f"[Interface] Failed to initialize MotionSwitcherClient: {e}")

        try:
            self._initLocoClient()
        except Exception as e:
            raise RuntimeError(f"[Interface] Failed to initialize LocoClient: {e}")


    def _initLowCmd(self):
        ''' Initialize the low-level command message with default values
            (stop motors and keep position) '''
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_cmd.mode_pr = self.data.MODE_PR # Series Control for Pitch/Roll Joints (default)
        self.low_cmd.mode_machine = 0            # Set on first incoming message
        for i in range(self.total_joint_num):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= 0
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = 0
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def _initLocoClient(self):
        self.lc = LocoClientG1()
        self.lc.SetTimeout(5.0)
        self.lc.Init()

    def _internalLowStateCallback(self, msg: LowState_G1):
        """Internal callback that executes class logic before user callback"""
        self.low_state_msg = msg

        # Check connection status on first message
        if not self.connected:
            self.connected = True
            self.mode_machine = self.low_state_msg.mode_machine # Set machine mode on first message
            self.low_cmd.mode_machine = self.mode_machine       # Update low cmd message as well
            print(f"[Interface] G1 Machine Mode: {self.mode_machine}, DOF: {self.data.MODE_MACHINE_DOF_DICT.get(self.mode_machine, 'MODE_MACHINE not recognized')}")
            if self.mode_machine == 6: # 27 DOF mode
                print("[Interface] G1 in 27 DOF mode, waist pitch and roll are phisically locked! Updating available motors list in the interaface.")
                self.available_motor_ids = [i for i in range(self.total_joint_num) if i not in [13,14]] # Exclude waist pitch and roll
                self.controlled_motor_ids = self.available_motor_ids.copy() # Update controlled motor IDs as well on first message (can be changed later by user)

        # Call the external low state message handler
        if self.low_state_callback is not None:
            self.low_state_callback()
        else:
            print("[Interface] No low state callback function provided.")

    # TODO move to parent class
    def LowCmdMotorUpdateControl(self,
                                  q: List[float], 
                                  dq: Union[float, List[float]], 
                                  tau: Union[float, List[float]],
                                  kp: Union[float, List[float]] = None, 
                                  kd: Union[float, List[float]] = None,
                                  override_controlled_motor_ids: Optional[List[int]] = None
                                  ):
        ''' Update low-level motor command values (for controlled motors by default, or override with specific motor IDs)
        
        Args:
            q: Target positions (must match length of motor_ids if provided)
            dq: Target velocities (scalar or list)
            tau: Target torques (scalar or list)
            kp: Position gains (scalar or list, uses defaults if None)
            kd: Velocity gains (scalar or list, uses defaults if None)
            override_controlled_motor_ids: Optional list of motor indices to update.
                                            If None uses the currently set controlled_motor_ids. Does not override the class variable.
        '''
        # Determine which motors to control
        if override_controlled_motor_ids is None:
            motor_ids_to_update = self.controlled_motor_ids
        else: # Only update specified motor IDs that are available
            motor_ids_to_update = [override_controlled_motor_ids[i] for i in range(len(override_controlled_motor_ids)) if override_controlled_motor_ids[i] in self.available_motor_ids]

        num_motors_to_update = len(motor_ids_to_update)

        # Convert inputs to lists if they are scalars
        if not (isinstance(q, list) or isinstance(q, np.ndarray)):
            print(f"[Interface] LowCmdMotorUpdateControl: q must be a list or numpy array")
            return
        if len(q) != num_motors_to_update:
            if len(q) == 29 and self.mode_machine == 6: # 27 DOF mode with full 29 DOF input
                print(f"[Interface] LowCmdMotorUpdateControl: Detected full 29 DoF q input in 27 DoF mode, extracting available motor values.")
                q = [q[i] for i in motor_ids_to_update]
            else:
                print(f"[Interface] LowCmdMotorUpdateControl: q length ({len(q)}) must match controlled_motor_ids length ({num_motors_to_update})")
                return
        
        if isinstance(dq, (int, float)): dq = [dq] * num_motors_to_update
        if isinstance(tau, (int, float)): tau = [tau] * num_motors_to_update
        
        if kp is None:
            kp = [self.kp_default[i] for i in motor_ids_to_update]
        if kd is None:
            kd = [self.kd_default[i] for i in motor_ids_to_update]

        with self.low_cmd_lock:
            for i, motor_id in enumerate(motor_ids_to_update):
                self.low_cmd.motor_cmd[motor_id].q = self._clampVariable(q[i], self.motor_model.min_position_list[motor_id], self.motor_model.max_position_list[motor_id])
                self.low_cmd.motor_cmd[motor_id].dq = self._clampVariable(dq[i], self.motor_model.min_velocity_list[motor_id], self.motor_model.max_velocity_list[motor_id])
                self.low_cmd.motor_cmd[motor_id].tau = self._clampVariable(tau[i], self.motor_model.min_torque_list[motor_id], self.motor_model.max_torque_list[motor_id])

                self.low_cmd.motor_cmd[motor_id].kp = kp[i]
                self.low_cmd.motor_cmd[motor_id].kd = kd[i]

    def LowCmdMotorSetControlledMotorIds(self, 
                                        controlled_motor_ids: List[int],
                                        default_q: Optional[List[float]] = None):
        ''' Set uncontrolled motors to a position control with default gains (keep_pos OR provided default_q) 
        while updating controlled motors normally.
        
        Args:
            controlled_motor_ids: List of motor IDs that are being actively controlled
            default_q: Optional list of default positions for all motors. 
                      If None, uses current position for unused motors.
        '''
        
        # Update controlled motor IDs and determine uncontrolled motors
        seen = set()
        unique_controlled = []
        for i in controlled_motor_ids:
            if i in self.available_motor_ids and i not in seen:
                seen.add(i)
            unique_controlled.append(i)
        self.controlled_motor_ids = unique_controlled
        uncontrolled_motor_ids = [i for i in self.available_motor_ids if i not in self.controlled_motor_ids]
        
        if len(uncontrolled_motor_ids) == 0:
            print("[Interface] All available motors are being controlled, no unused motors to set.")
            return
        
        # Get default positions for unused motors
        if default_q is None:
            # Use current positions
            q_unused = [self.low_state_msg.motor_state[i].q for i in uncontrolled_motor_ids]
            print(f"[Interface] Setting unused motors {uncontrolled_motor_ids} to current positions.")
        else:
            q_unused = [default_q[i] for i in uncontrolled_motor_ids]
            print(f"[Interface] Setting unused motors {uncontrolled_motor_ids} to provided default positions.")
        
        # Set unused motors to default position with zero velocity/torque
        self.LowCmdMotorUpdateControl(
            q=q_unused,
            dq=0.0,
            tau=0.0,
            kp=None,  # Use default kp
            kd=None,  # Use default kd
            override_controlled_motor_ids=uncontrolled_motor_ids
        )

    # TODO move to parent class
    def _lowCmdWrite(self):
        ''' Publish low-level motor commands to the robot '''
        if self.emergency_event.is_set():
            self.msc.ReleaseMode()
            print("EMERGENCY STOP activated, sent ReleaseMode command to robot.")

        if self.use_interp.is_set():
            self._lowCmdMotorUpdateInterpLoop()

        # self._printLowCmd()  # DEBUG

        with self.low_cmd_lock:
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_pub.Write(self.low_cmd)

    # TODO move to parent class
    def _lowCmdMotorUpdateInterpLoop(self): 
        prev_targets = self.interp_targets.get_prev_targets()
        next_targets = self.interp_targets.get_next_targets()

        self.interp_progress += 1.0 / self.interp_total_step
        self.interp_progress = min(self.interp_progress, 1)

        q = [(1 - self.interp_progress) * prev_targets[0][i] + self.interp_progress * next_targets[0][i] for i in range(self.total_joint_num)]
        dq = 0.0
        kp = self.kp_default  # ADD kp and kd to interp handling class !!! otherwise it cant be changed here
        kd = self.kd_default
        tau = 0.0
        self.LowCmdMotorUpdateControl(q, dq, tau, kp, kd)
        # print(f"Interp progress: {self.interp_progress:.3f}", f"{q[:3]}") # DEBUG 

        if self.interp_progress == 1.0 and self.going_init_pos.is_set():
            print("[Interface] Reached init target position.")
            self.interp_total_step = 40
            self.going_init_pos.clear()
            self.use_interp.clear()

        if self.interp_progress == 1.0 and self.going_stopping_pos.is_set():
            print("[Interface] Reached stop target position.")
            self.interp_total_step = 10
            self.use_interp.clear()
            self._stopLowCmdWriteThread()
            # self.going_stopping_pos.clear()

    # TODO move to parent class
    def LowCmdMotorUpdateInterpTarget(self, tar_q=None, tar_dq=None, tar_tau=None, kp=None, kd=None, set_cur_pos_as_target=False):
        ''' Update low-level motor command values for target position control'''

        if not self.going_init_pos.is_set() and not self.going_stopping_pos.is_set():
            if set_cur_pos_as_target: # Initialize interpolation targets
                current_q = [ms.q for ms in self.low_state_msg.motor_state]
                controlled_motor_num = len(self.controlled_motor_ids)
                self.interp_targets.set_prev_targets(current_q, [0] * controlled_motor_num, [0] * controlled_motor_num) # if first run set current pos as prev pos
                if not tar_q:
                    self.interp_targets.set_next_targets(current_q, [0] * controlled_motor_num, [0] * controlled_motor_num) # if no target pos given set current pos as target pos
                else: 
                    self.interp_targets.set_next_targets(tar_q, [0]*controlled_motor_num, [0]*controlled_motor_num)
                self.interp_progress = 0.0
                
            elif tar_q != None and tar_dq != None and tar_tau != None: # Set new target pos
                self.interp_targets.shift_targets()
                self.interp_targets.set_next_targets(tar_q, tar_dq, tar_tau)
                self.interp_progress = 0.0

    # TODO adapt to G1
    def StartLowCmdControl(self, keep_still:bool = True,
                           target_pose:str = None,
                           target_q: list = None):
        ''' Start the low-level command control thread while keeping the robot in current position or interpolating to target position '''
        
        # Wait for connection before starting control to determine machine mode and available motors
        while not self.connected:
            print("[Interface] Waiting for connection to robot before starting low-level command control.")
            time.sleep(1.0)
        
        if keep_still:
            current_q = [ms.q for ms in self.low_state_msg.motor_state]
            current_q = current_q[:self.total_joint_num]
            self.LowCmdMotorUpdateControl(current_q, 0.0, 0.0)
            print('[Interface] Starting low-level command control, keeping current position.')
        else:
            if target_q is None:
                target_q = self.data.q_standing
            self.interp_total_step = 600
            self.LowCmdMotorUpdateInterpTarget(tar_q=target_q, set_cur_pos_as_target=True)
            self.use_interp.set()
            self.going_init_pos.set()
            print('[Interface] Starting low-level command control, interpolating to target position.')
        
        self._startLowCmdWriteThread() # Start publishing low-level commands after the initial target posion is set
        # RESET INTERP TOTAL STEP TO DEFAULT

    # TODO adapt to G1
    def StopLowCmdControl(self, stop_target_q: list = None):
        if stop_target_q is None:
            stop_target_q = None # TODO define a lying down position for G1 or use high level command to lie down
        self.interp_total_step = 600
        self.LowCmdMotorUpdateInterpTarget(tar_q=stop_target_q, set_cur_pos_as_target=True)
        self.use_interp.set()
        self.going_stopping_pos.set()
        print('[Interface] Starting stopping process, interpolating to stop position.')


    def CloseConnection(self):
        super().CloseConnection()
        self.StopLowCmdControl()
        # self._stopLowCmdMotorPublishThread() # Graceful shutdown not working yet
