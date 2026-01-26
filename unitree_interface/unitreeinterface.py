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

class InterpTargets:
    def __init__(self):
        self.prev_q = [0.0]*12
        self.prev_dq = [0.0]*12
        self.prev_tau = [0.0]*12

        self.next_q = [0.0]*12
        self.next_dq = [0.0]*12
        self.next_tau = [0.0]*12

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
        self.interp_targets = InterpTargets()

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
        while self.connected == False:
            print("[Interface] Waiting for connection to robot before starting low-level command thread.")
            time.sleep(1.0)

        if self.connected:
            print("[Interface] Starting low-level command write thread.")
            self.low_cmd_write_thread.Start()

    def _stopLowCmdWriteThread(self):
        if self.low_cmd_write_thread is not None:
            print("[Interface] Stopping low-level command write thread.")
            self.low_cmd_write_thread.Wait(timeout=1.0)

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


@dataclass
class UnitreeInterfaceDataGO2:
    HIGHLEVEL = 0xEE
    LOWLEVEL = 0xFF
    TRIGERLEVEL = 0xF0
    PosStopF = 2.146e9
    VelStopF = 16000.0
    pos_lying = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65,]
    pos_standing = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]

class UnitreeInterfaceGO2(UnitreeInterface):
    def __init__(self,
                 network_interface: str,
                 low_state_callback = None,
                 low_cmd_pub_dt: float = 0.002):
        super().__init__(network_interface, low_state_callback, low_cmd_pub_dt)
        
        self.kp_default = [60.0] * 12
        self.kd_default = [5.0] * 12
        self.go2_motor_model = Go2MotorModel()

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
            print(f"Failed to initialize SportClient: {e}")
        try:
            self._initMotionSwitcherClient()
        except Exception as e:
            print(f"Failed to initialize MotionSwitcherClient: {e}")

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
        if isinstance(dq, (int, float)): dq = [dq] * 12
        if isinstance(tau, (int, float)): tau = [tau] * 12
        
        if kp is None:
            kp = self.kp_default
        elif isinstance(kp, (int, float)):
            kp = [kp] * 12
        if kd is None:
            kd = self.kd_default
        elif isinstance(kd, (int, float)):
            kd = [kd] * 12
        
        min_q   = self.go2_motor_model.get_vec_var("min_position")
        max_q   = self.go2_motor_model.get_vec_var("max_position")
        min_dq  = self.go2_motor_model.get_vec_var("min_velocity")
        max_dq  = self.go2_motor_model.get_vec_var("max_velocity")
        min_tau = self.go2_motor_model.get_vec_var("min_torque")
        max_tau = self.go2_motor_model.get_vec_var("max_torque")

        with self.low_cmd_lock:
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self._clampVariable(q[i], min_q[i], max_q[i])
                self.low_cmd.motor_cmd[i].dq = self._clampVariable(dq[i], min_dq[i], max_dq[i])
                self.low_cmd.motor_cmd[i].tau = self._clampVariable(tau[i], min_tau[i], max_tau[i])

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

        #print(f"Publishing low-level command q: {[self.low_cmd.motor_cmd[i].q for i in range(12)]}") # DEBUG

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
            print("[Interface] ]Reached init target position.")
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
                self.interp_targets.set_prev_targets(current_q, [0] * 12, [0] * 12) # if first run set current pos as prev pos
                if not tar_q:
                    self.interp_targets.set_next_targets(current_q, [0] * 12, [0] * 12) # if no target pos given set current pos as target pos
                else: 
                    self.interp_targets.set_next_targets(tar_q, [0]*12, [0]*12)
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
                target_q = self.data.pos_standing
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
    MODE_PR = 0 # Series Control for Pitch/Roll Joints
    MODE_AB = 1 # Parallel Control for A/B Joints
    MODE_MACHINE = None # G1 Type：4：23-Dof;5:29-Dof;6:27-Dof(29Dof Fitted at the waist)
    PosStopF = 2.146e9
    VelStopF = 16000.0
    pos_standing = []

class UnitreeInterfaceG1(UnitreeInterface):
    def __init__(self,
                 network_interface: str,
                 low_state_callback = None,
                 low_cmd_pub_dt: float = 0.002):
        super().__init__(network_interface, low_state_callback, low_cmd_pub_dt)
        
        self.kp_default = [
            60, 60, 60, 100, 40, 40,      # legs
            60, 60, 60, 100, 40, 40,      # legs
            60, 40, 40,                   # waist
            40, 40, 40, 40,  40, 40, 40,  # arms
            40, 40, 40, 40,  40, 40, 40   # arms
        ]
        self.kd_default = [
            1, 1, 1, 2, 1, 1,     # legs
            1, 1, 1, 2, 1, 1,     # legs
            1, 1, 1,              # waist
            1, 1, 1, 1, 1, 1, 1,  # arms
            1, 1, 1, 1, 1, 1, 1   # arms
        ]
        self.g1_motor_model = G1MotorModel()

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
            self._initLocoClient()
        except Exception as e:
            print(f"Failed to initialize LocoClient: {e}")
        try:
            self._initMotionSwitcherClient()
        except Exception as e:
            print(f"Failed to initialize MotionSwitcherClient: {e}")

        self.mode_machine = None # Machine mode (rewritten on first incoming message)

    def _initLowCmd(self):
        ''' Initialize the low-level command message with default values
            (stop motors and keep position) '''
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_cmd.mode_pr = self.data.MODE_AB # Parellel Control for A/B Joints
        self.low_cmd.mode_machine = None         # Set on first incoming message
        for i in range(29):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= self.data.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = self.data.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def _initLocoClient(self):
        self.lc = LocoClientG1()
        self.lc.SetTimeout(5.0)
        self.lc.Init()

    # TODO move to parent class
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
        if isinstance(dq, (int, float)): dq = [dq] * 12
        if isinstance(tau, (int, float)): tau = [tau] * 12
        
        if kp is None:
            kp = self.kp_default
        elif isinstance(kp, (int, float)):
            kp = [kp] * 12
        if kd is None:
            kd = self.kd_default
        elif isinstance(kd, (int, float)):
            kd = [kd] * 12
        
        min_q   = self.go2_motor_model.get_vec_var("min_position")
        max_q   = self.go2_motor_model.get_vec_var("max_position")
        min_dq  = self.go2_motor_model.get_vec_var("min_velocity")
        max_dq  = self.go2_motor_model.get_vec_var("max_velocity")
        min_tau = self.go2_motor_model.get_vec_var("min_torque")
        max_tau = self.go2_motor_model.get_vec_var("max_torque")

        with self.low_cmd_lock:
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self._clampVariable(q[i], min_q[i], max_q[i])
                self.low_cmd.motor_cmd[i].dq = self._clampVariable(dq[i], min_dq[i], max_dq[i])
                self.low_cmd.motor_cmd[i].tau = self._clampVariable(tau[i], min_tau[i], max_tau[i])

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

        #print(f"Publishing low-level command q: {[self.low_cmd.motor_cmd[i].q for i in range(12)]}") # DEBUG

        with self.low_cmd_lock:
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_pub.Write(self.low_cmd)

    # TODO move to parent class
    def _lowCmdMotorUpdateInterpLoop(self): 
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
            print("[Interface] ]Reached init target position.")
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
                self.interp_targets.set_prev_targets(current_q, [0] * 12, [0] * 12) # if first run set current pos as prev pos
                if not tar_q:
                    self.interp_targets.set_next_targets(current_q, [0] * 12, [0] * 12) # if no target pos given set current pos as target pos
                else: 
                    self.interp_targets.set_next_targets(tar_q, [0]*12, [0]*12)
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
        if keep_still:
            current_q = [ms.q for ms in self.low_state_msg.motor_state]
            
            self.LowCmdMotorUpdateControl(current_q, 0.0, 0.0)
            print('[Interface] Starting low-level command control, keeping current position.')
        else:
            if target_q is None:
                target_q = self.data.pos_standing
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
class MotorModel:
    """Unitree motor model for storing motor limits"""
    def __init__(
      self,
      name: Optional[str] = None,
      min_position: float = 0.0,
      max_position: float = 0.0,
      min_velocity: float = 0.0,
      max_velocity: float = 0.0,
      min_torque: float = 0.0,
      max_torque: float = 0.0,) -> None:

        self.name = name
        self.min_position = min_position
        self.max_position = max_position
        self.min_velocity = min_velocity
        self.max_velocity = max_velocity
        self.min_torque = min_torque
        self.max_torque = max_torque


@dataclass
class Go2MotorModel(MotorModel):
    """Go2 robot model for real applications"""
    def __init__(self):
        self.motors = [
            MotorModel(
                name="FR_hip_joint",
                min_position=-1.05,
                max_position=1.05,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="FR_thigh_joint",
                min_position=-1.57,
                max_position=3.49,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="FR_calf_joint",
                min_position=-2.818,
                max_position=-0.838,
                min_velocity=-20,
                max_velocity=20,
                min_torque=-45.4,
                max_torque=45.4,
            ),
            MotorModel(
                name="FL_hip_joint",
                min_position=-1.05,
                max_position=1.05,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="FL_thigh_joint",
                min_position=-1.57,
                max_position=3.49,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="FL_calf_joint",
                min_position=-2.818,
                max_position=-0.838,
                min_velocity=-20,
                max_velocity=20,
                min_torque=-45.4,
                max_torque=45.4,
            ),
            MotorModel(
                name="RR_hip_joint",
                min_position=-1.05,
                max_position=1.05,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="RR_thigh_joint",
                min_position=-0.524,
                max_position=4.54,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="RR_calf_joint",
                min_position=-2.818,
                max_position=-0.838,
                min_velocity=-20,
                max_velocity=20,
                min_torque=-45.4,
                max_torque=45.4,
            ),
            MotorModel(
                name="RL_hip_joint",
                min_position=-1.05,
                max_position=1.05,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="RL_thigh_joint",
                min_position=-0.524,
                max_position=4.54,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            ),
            MotorModel(
                name="RL_calf_joint",
                min_position=-2.818,
                max_position=-0.838,
                min_velocity=-20,
                max_velocity=20,
                min_torque=-45.4,
                max_torque=45.4,
            )]
    
    def get_vec_var(self, var_name: Literal["name", "min_position", "max_position", "min_velocity", "max_velocity", "min_torque", "max_torque"]):
        return [getattr(motor, var_name) for motor in self.motors]

@dataclass
class G1MotorModel(MotorModel):
    """Go2 robot model for real applications"""
    def __init__(self):
        self.motors = [
            MotorModel(
                name="FR_hip_joint",
                min_position=-1.05,
                max_position=1.05,
                min_velocity=-30,
                max_velocity=30,
                min_torque=-23.7,
                max_torque=23.7,
            )]

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof


