from dataclasses import dataclass
from typing import Union, List, Optional, Callable, Literal

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

    def __str__(self):
        motor_str = 'MotorModel(\n    name="' + str(self.name) + '",\n'
        motor_str += "    min_position=" + str(self.min_position) + ",\n    max_position=" + str(self.max_position) + ",\n"
        motor_str += "    min_velocity=" + str(self.min_velocity) + ",\n    max_velocity=" + str(self.max_velocity) + ",\n"
        motor_str += "    min_torque=" + str(self.min_torque) + ",\n    max_torque=" + str(self.max_torque) + "\n),"
        return motor_str
    
    def set_attrib_lists(self,motors_list):
        self.name_list = [motor.name for motor in motors_list]
        self.min_position_list = [motor.min_position for motor in motors_list]
        self.max_position_list = [motor.max_position for motor in motors_list]
        self.min_velocity_list = [motor.min_velocity for motor in motors_list]
        self.max_velocity_list = [motor.max_velocity for motor in motors_list]
        self.min_torque_list = [motor.min_torque for motor in motors_list]
        self.max_torque_list = [motor.max_torque for motor in motors_list]
    

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
        self.set_attrib_lists(self.motors)
    

@dataclass
class G1MotorModel(MotorModel):
    """G1 robot model for real applications (indices validated from unitree docs)"""
    def __init__(self):
        # Indices from the docs
        self.LeftHipPitch = 0
        self.LeftHipRoll = 1
        self.LeftHipYaw = 2
        self.LeftKnee = 3
        self.LeftAnklePitch = 4
        #self.LeftAnkleB = 4
        self.LeftAnkleRoll = 5
        #self.LeftAnkleA = 5
        self.RightHipPitch = 6
        self.RightHipRoll = 7
        self.RightHipYaw = 8
        self.RightKnee = 9
        self.RightAnklePitch = 10
        #self.RightAnkleB = 10
        self.RightAnkleRoll = 11
        #self.RightAnkleA = 11
        self.WaistYaw = 12
        self.WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
        #self.WaistA = 13          # NOTE: INVALID for g1 23dof/29dof with waist locked
        self.WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
        #self.WaistB = 14          # NOTE: INVALID for g1 23dof/29dof with waist locked
        self.LeftShoulderPitch = 15
        self.LeftShoulderRoll = 16
        self.LeftShoulderYaw = 17
        self.LeftElbow = 18
        self.LeftWristRoll = 19
        self.LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
        self.LeftWristYaw = 21     # NOTE: INVALID for g1 23dof
        self.RightShoulderPitch = 22
        self.RightShoulderRoll = 23
        self.RightShoulderYaw = 24
        self.RightElbow = 25
        self.RightWristRoll = 26
        self.RightWristPitch = 27  # NOTE: INVALID for g1 23dof
        self.RightWristYaw = 28    # NOTE: INVALID for g1 23dof

        self.motors = [
            MotorModel(
                name="left_hip_pitch_joint",
                min_position=-2.5307,
                max_position=2.8798,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.5307,
                max_torque=2.8798
            ),
            MotorModel(
                name="left_hip_roll_joint",
                min_position=-0.5236,
                max_position=2.9671,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.5236000000000001,
                max_torque=2.9671
            ),
            MotorModel(
                name="left_hip_yaw_joint",
                min_position=-2.7576,
                max_position=2.7576,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.7576,
                max_torque=2.7576
            ),
            MotorModel(
                name="left_knee_joint",
                min_position=-0.087267,
                max_position=2.8798,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.0872670000000002,
                max_torque=2.8798
            ),
            MotorModel(
                name="left_ankle_pitch_joint",
                min_position=-0.87267,
                max_position=0.5236,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.87267,
                max_torque=0.5236
            ),
            MotorModel(
                name="left_ankle_roll_joint",
                min_position=-0.2618,
                max_position=0.2618,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.2618,
                max_torque=0.2618
            ),
            MotorModel(
                name="right_hip_pitch_joint",
                min_position=-2.5307,
                max_position=2.8798,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.5307,
                max_torque=2.8798
            ),
            MotorModel(
                name="right_hip_roll_joint",
                min_position=-2.9671,
                max_position=0.5236,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.9671,
                max_torque=0.5236000000000001
            ),
            MotorModel(
                name="right_hip_yaw_joint",
                min_position=-2.7576,
                max_position=2.7576,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.7576,
                max_torque=2.7576
            ),
            MotorModel(
                name="right_knee_joint",
                min_position=-0.087267,
                max_position=2.8798,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.0872670000000002,
                max_torque=2.8798
            ),
            MotorModel(
                name="right_ankle_pitch_joint",
                min_position=-0.87267,
                max_position=0.5236,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.87267,
                max_torque=0.5236
            ),
            MotorModel(
                name="right_ankle_roll_joint",
                min_position=-0.2618,
                max_position=0.2618,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.2618,
                max_torque=0.2618
            ),
            MotorModel(
                name="waist_yaw_joint",
                min_position=-2.618,
                max_position=2.618,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.618,
                max_torque=2.618
            ),
            MotorModel(
                name="waist_roll_joint",
                min_position=-0.52,
                max_position=0.52,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.52,
                max_torque=0.52
            ),
            MotorModel(
                name="waist_pitch_joint",
                min_position=-0.52,
                max_position=0.52,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-0.52,
                max_torque=0.52
            ),
            MotorModel(
                name="left_shoulder_pitch_joint",
                min_position=-3.0892,
                max_position=2.6704,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-3.0892,
                max_torque=2.6704
            ),
            MotorModel(
                name="left_shoulder_roll_joint",
                min_position=-1.5882,
                max_position=2.2515,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.5882,
                max_torque=2.2515
            ),
            MotorModel(
                name="left_shoulder_yaw_joint",
                min_position=-2.618,
                max_position=2.618,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.618,
                max_torque=2.618
            ),
            MotorModel(
                name="left_elbow_joint",
                min_position=-1.0472,
                max_position=2.0944,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.0471999999999997,
                max_torque=2.0944
            ),
            MotorModel(
                name="left_wrist_roll_joint",
                min_position=-1.97222,
                max_position=1.97222,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.97222,
                max_torque=1.97222
            ),
            MotorModel(
                name="left_wrist_pitch_joint",
                min_position=-1.61443,
                max_position=1.61443,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.61443,
                max_torque=1.61443
            ),
            MotorModel(
                name="left_wrist_yaw_joint",
                min_position=-1.61443,
                max_position=1.61443,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.61443,
                max_torque=1.61443
            ),
            MotorModel(
                name="right_shoulder_pitch_joint",
                min_position=-3.0892,
                max_position=2.6704,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-3.0892,
                max_torque=2.6704
            ),
            MotorModel(
                name="right_shoulder_roll_joint",
                min_position=-2.2515,
                max_position=1.5882,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.2515,
                max_torque=1.5882
            ),
            MotorModel(
                name="right_shoulder_yaw_joint",
                min_position=-2.618,
                max_position=2.618,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-2.618,
                max_torque=2.618
            ),
            MotorModel(
                name="right_elbow_joint",
                min_position=-1.0472,
                max_position=2.0944,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.0471999999999997,
                max_torque=2.0944
            ),
            MotorModel(
                name="right_wrist_roll_joint",
                min_position=-1.97222,
                max_position=1.97222,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.97222,
                max_torque=1.97222
            ),
            MotorModel(
                name="right_wrist_pitch_joint",
                min_position=-1.61443,
                max_position=1.61443,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.61443,
                max_torque=1.61443
            ),
            MotorModel(
                name="right_wrist_yaw_joint",
                min_position=-1.61443,
                max_position=1.61443,
                min_velocity=-50.0,
                max_velocity=50.0,
                min_torque=-1.61443,
                max_torque=1.61443
            ),
        ]
        self.set_attrib_lists(self.motors)

def getMotorDataFromMujocoModel(xml_path: str,
                                fallback_motor_model: MotorModel) -> List[MotorModel]:
    """Extracts motor names and constraints from mujoco xml file if Mujoco is installed."""
    try: 
        import mujoco
    except ImportError:
        print("Mujoco not installed, falling back to predefined limits")
        return fallback_motor_model().motors
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    motor_models = []
    try:
        for i in range(model.nu):
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            joint_id = model.actuator_trnid[i, 0]

            min_pos = model.jnt_range[joint_id, 0]
            max_pos = model.jnt_range[joint_id, 1]
            min_tau = model.actuator_ctrlrange[i, 0]
            max_tau = model.actuator_ctrlrange[i, 1]
            min_vel = -50.0 # default values as mujoco does not provide velocity limits
            max_vel =  50.0
        
            motor_models.append(MotorModel(
                                    name=actuator_name,
                                    min_position=min_pos,
                                    max_position=max_pos,
                                    min_velocity=min_vel,
                                    max_velocity=max_vel,
                                    min_torque=min_tau,
                                    max_torque=max_tau))
    except:
        print("Problem encountered, falling back to fallback motor model")
        return fallback_motor_model().motors
    
    return motor_models

def getJointMapping(xml_path: str,
                    reference_motor_model: MotorModel) -> List[int]:
    """
    Maps joints from a MuJoCo XML model to a reference motor model.
    Returns a list of joint indices found in the XML that match the reference model.
    
    Args:
        xml_path: Path to the MuJoCo XML file
        reference_motor_model: Reference motor model class (e.g., G1MotorModel)
    
    Returns:
        List of joint indices that exist in both the XML and reference model
    """
    try:
        import mujoco
    except ImportError:
        print("Mujoco not installed, cannot create joint mapping")
        return []
    
    # Get reference motor names
    ref_motors = reference_motor_model().motors
    ref_motor_names = [motor.name for motor in ref_motors]
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    xml_motor_names = []
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        xml_motor_names.append(actuator_name)
    
    joint_mapping = []
    
    # Print table
    print("\n" + "="*80)
    print(f"{'Index':<8} {'Joint Name':<35} {'In Model':<10}")
    print("="*80)
    
    for idx, ref_name in enumerate(ref_motor_names):
        is_in_model = ref_name in xml_motor_names
        status = "YES" if is_in_model else "NO"
        
        print(f"{idx:<8} {ref_name:<35} {status:<10}")
        
        if is_in_model:
            joint_mapping.append(idx)
    
    print("="*80)
    print(f"Total joints in reference model: {len(ref_motor_names)}")
    print(f"Joints found in XML model: {len(joint_mapping)}")
    print(f"Joint indices to use: {joint_mapping}")
    print("="*80 + "\n")
    
    return joint_mapping


if __name__ == "__main__":
    motors = getMotorDataFromMujocoModel("/home/adamk/code/dial-mpc/dial_mpc/models/unitree_g1/g1_mjx_29dof.xml",
                                      fallback_motor_model=G1MotorModel)
    #for motor in motors:
    #    print(motor)

    joint_mapping = getJointMapping("/home/adamk/code/dial-mpc/dial_mpc/models/unitree_g1/g1_mjx_21dof.xml",
                                     reference_motor_model=G1MotorModel)