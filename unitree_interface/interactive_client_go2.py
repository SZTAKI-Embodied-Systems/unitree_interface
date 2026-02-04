import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
import os
import threading
import termios
import tty
import select
import atexit
from enum import Enum


class MockSportClient:
    """Mock SportClient for debug mode - simulates robot commands without connection"""
    def SetTimeout(self, timeout):
        print(f"[DEBUG] SetTimeout({timeout})")
    
    def Init(self):
        print(f"[DEBUG] Init()")
    
    def Damp(self):
        print(f"[DEBUG] Damp() - Emergency damping")
        return 0
    
    def StandUp(self):
        print(f"[DEBUG] StandUp()")
        return 0
    
    def StandDown(self):
        print(f"[DEBUG] StandDown()")
        return 0
    
    def Move(self, x, y, z):
        print(f"[DEBUG] Move(x={x}, y={y}, z={z})")
        return 0
    
    def StopMove(self):
        print(f"[DEBUG] StopMove()")
        return 0
    
    def FreeWalk(self):
        print(f"[DEBUG] FreeWalk()")
        return 0


class RobotState(Enum):
    """State machine states for robot control"""
    INIT = "INIT"
    WAIT = "WAIT"
    STANDUP = "STANDUP"
    STANDDOWN = "STANDDOWN"
    FREEWALK = "FREEWALK"
    WAITFORMOVE = "WAITFORMOVE"
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    LEFT_TURN = "LEFT_TURN"
    RIGHT_TURN = "RIGHT_TURN"


class KeyboardHandler:
    """Handles keyboard input with continuous key detection"""
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._orig_termios = termios.tcgetattr(self._fd)
        self._current_key = None
        self._running = False
        self._lock = threading.Lock()
        
        atexit.register(self.restore_terminal)

    def start(self):
        """Start keyboard monitoring in raw mode"""
        self._running = True
        tty.setcbreak(self._fd)
        
    def stop(self):
        """Stop keyboard monitoring and restore terminal"""
        self._running = False
        self.restore_terminal()
        
    def get_key(self, timeout=0.1):
        """
        Get currently pressed key with timeout.
        Returns the key character or None if no key pressed.
        """
        if not self._running:
            return None
            
        r, w, e = select.select([self._fd], [], [], timeout)
        if r:
            ch = os.read(self._fd, 1).decode()
            with self._lock:
                self._current_key = ch
            print(f"[KEY] Detected key: {repr(ch)}")
            return ch
        return None
    
    def is_key_pressed(self, key):
        """Check if a specific key is currently pressed"""
        with self._lock:
            return self._current_key == key
    
    def clear_key(self):
        """Clear the current key state"""
        with self._lock:
            self._current_key = None
            
    def restore_terminal(self):
        """Restore terminal to original settings"""
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._orig_termios)
        except:
            pass


class RobotStateMachine:
    """State machine for interactive robot control"""
    def __init__(self, sport_client, debug_mode=False):
        self.sport_client = sport_client
        self.state = RobotState.INIT
        self.keyboard = KeyboardHandler()
        self.running = True
        self.debug_mode = debug_mode
        
    def run(self):
        """Main state machine loop"""
        self.keyboard.start()
        
        print("\n" + "="*60)
        print("ROBOT INTERACTIVE CONTROL")
        print("="*60)
        print("\nControls:")
        print("  INIT State:")
        print("    - Press ENTER to start")
        print("\n  WAIT State:")
        print("    - 8: Stand Up")
        print("    - 2: Stand Down")
        print("    - F: Enter Free Walk mode")
        print("\n  FREEWALK State:")
        print("    - Activates Free Walk, then waits 1s")
        print("    - Auto-transitions to WAITFORMOVE")
        print("\n  WAITFORMOVE State:")
        print("    - W: Forward")
        print("    - S: Backward")
        print("    - A: Left")
        print("    - D: Right")
        print("    - 4: Turn Left")
        print("    - 6: Turn Right")
        print("    - 8: Stand Up")
        print("    - 2: Stand Down")
        print("\n  Global Commands:")
        print("    - Q: Emergency Damping")
        print("    - E: Stop Movement")
        print("    - ESC: Exit program")
        print("="*60 + "\n")
        
        try:
            while self.running:
                key = self.keyboard.get_key(timeout=0.05)
                
                # Global emergency commands (work in any state)
                if key:
                    if key.lower() == 'q':
                        print("\n[EMERGENCY] Damping activated!")
                        self.sport_client.Damp()
                        print("[EMERGENCY] Exiting program...")
                        self.running = False
                        break
                    elif key.lower() == 'r':
                        print("\n[STOP] Stop movement command sent!")
                        self.sport_client.StopMove()
                        time.sleep(0.5)
                        continue
                    elif key == '\x1b':  # ESC key
                        print("\n[EXIT] Exiting program...")
                        self.running = False
                        break
                
                # State machine logic
                if self.state == RobotState.INIT:
                    self._handle_init(key)
                elif self.state == RobotState.WAIT:
                    self._handle_wait(key)
                elif self.state == RobotState.STANDUP:
                    self._handle_standup()
                elif self.state == RobotState.STANDDOWN:
                    self._handle_standdown()
                elif self.state == RobotState.FREEWALK:
                    self._handle_freewalk(key)
                elif self.state == RobotState.WAITFORMOVE:
                    self._handle_waitformove(key)
                elif self.state in [RobotState.FORWARD, RobotState.BACKWARD, 
                                   RobotState.LEFT, RobotState.RIGHT,
                                   RobotState.LEFT_TURN, RobotState.RIGHT_TURN]:
                    self._handle_movement(key)
                    
                time.sleep(0.05)  # Small delay to prevent CPU spinning
                
        except KeyboardInterrupt:
            print("\n[INTERRUPT] Shutting down...")
        finally:
            self.keyboard.stop()
            print("[EXIT] Control session ended.")
    
    def _handle_init(self, key):
        """Handle INIT state"""
        if key == '\r' or key == '\n':  # Enter key
            print(f"[STATE] INIT -> WAIT")
            self.state = RobotState.WAIT
        elif self.state == RobotState.INIT:
            print("[INIT] Press ENTER to start...", end='\r')
    
    def _handle_wait(self, key):
        """Handle WAIT state"""
        if key == '8':  # Up (numpad 8)
            print(f"\n[STATE] WAIT -> STANDUP")
            self.state = RobotState.STANDUP
        elif key == '2':  # Down (numpad 2)
            print(f"\n[STATE] WAIT -> STANDDOWN")
            self.state = RobotState.STANDDOWN
        elif key and key.lower() == 'f':
            print(f"\n[STATE] WAIT -> FREEWALK")
            self.state = RobotState.FREEWALK
    
    def _handle_standup(self):
        """Handle STANDUP state"""
        print("[ACTION] Standing up...")
        self.sport_client.StandUp()
        time.sleep(2.0)
        print(f"[STATE] STANDUP -> WAIT")
        self.state = RobotState.WAIT
    
    def _handle_standdown(self):
        """Handle STANDDOWN state"""
        print("[ACTION] Standing down...")
        self.sport_client.StandDown()
        time.sleep(2.0)
        print(f"[STATE] STANDDOWN -> WAIT")
        self.state = RobotState.WAIT
    
    def _handle_waitformove(self, key):
        """Handle WAITFORMOVE state - wait for movement commands"""
        if key:
            if key == '8':  # Up (numpad 8)
                print(f"\n[STATE] WAITFORMOVE -> STANDUP")
                self.state = RobotState.STANDUP
            elif key == '2':  # Down (numpad 2)
                print(f"\n[STATE] WAITFORMOVE -> STANDDOWN")
                self.state = RobotState.STANDDOWN
            elif key == '4':  # Left turn (numpad 4)
                print(f"\n[STATE] WAITFORMOVE -> LEFT_TURN")
                self.state = RobotState.LEFT_TURN
            elif key == '6':  # Right turn (numpad 6)
                print(f"\n[STATE] WAITFORMOVE -> RIGHT_TURN")
                self.state = RobotState.RIGHT_TURN
            elif key.lower() == 'w':
                print(f"\n[STATE] WAITFORMOVE -> FORWARD")
                self.state = RobotState.FORWARD
            elif key.lower() == 's':
                print(f"\n[STATE] WAITFORMOVE -> BACKWARD")
                self.state = RobotState.BACKWARD
            elif key.lower() == 'a':
                print(f"\n[STATE] WAITFORMOVE -> LEFT")
                self.state = RobotState.LEFT
            elif key.lower() == 'd':
                print(f"\n[STATE] WAITFORMOVE -> RIGHT")
                self.state = RobotState.RIGHT

    def _handle_freewalk(self, key):
        """Handle FREEWALK state - activate free walk and transition to WAITFORMOVE"""
        print("[ACTION] Activating Free Walk mode...")
        ret = self.sport_client.FreeWalk()
        print(f"[FREEWALK] Return code: {ret}")
        time.sleep(1.0)
        print(f"[STATE] FREEWALK -> WAITFORMOVE")
        self.state = RobotState.WAITFORMOVE
    
    def _handle_movement(self, key):
        """Handle movement states with continuous key detection"""
        # Execute the movement command based on current state
        if self.state == RobotState.FORWARD:
            print("[ACTION] Moving forward...")
            self.sport_client.Move(0.3, 0, 0)
            required_key = 'w'
        elif self.state == RobotState.BACKWARD:
            print("[ACTION] Moving backward...")
            self.sport_client.Move(-0.3, 0, 0)
            required_key = 's'
        elif self.state == RobotState.LEFT:
            print("[ACTION] Moving left...")
            self.sport_client.Move(0, 0.3, 0)
            required_key = 'a'
        elif self.state == RobotState.RIGHT:
            print("[ACTION] Moving right...")
            self.sport_client.Move(0, -0.3, 0)
            required_key = 'd'
        elif self.state == RobotState.LEFT_TURN:
            print("[ACTION] Turning left...")
            self.sport_client.Move(0, 0, 0.5)
            required_key = '4'  # Numpad 4
        elif self.state == RobotState.RIGHT_TURN:
            print("[ACTION] Turning right...")
            self.sport_client.Move(0, 0, -0.5)
            required_key = '6'  # Numpad 6
        
        # Wait 1 second while monitoring for key presses
        start_time = time.time()
        new_key = None
        while time.time() - start_time < 1.0:
            new_key = self.keyboard.get_key(timeout=0.05)
            if new_key:
                break
            time.sleep(0.05)
        
        # Check what to do after 1 second
        if new_key:
            # Check for numeric keys (8, 2, 4, 6)
            if new_key == '8':  # Up (numpad 8)
                print(f"\n[STATE] {self.state.value} -> STANDUP")
                self.state = RobotState.STANDUP
                return
            elif new_key == '2':  # Down (numpad 2)
                print(f"\n[STATE] {self.state.value} -> STANDDOWN")
                self.state = RobotState.STANDDOWN
                return
            elif new_key == '4' and self.state != RobotState.LEFT_TURN:  # Left turn
                print(f"\n[STATE] {self.state.value} -> LEFT_TURN")
                self.state = RobotState.LEFT_TURN
                return
            elif new_key == '6' and self.state != RobotState.RIGHT_TURN:  # Right turn
                print(f"\n[STATE] {self.state.value} -> RIGHT_TURN")
                self.state = RobotState.RIGHT_TURN
                return
            elif (new_key == '4' and self.state == RobotState.LEFT_TURN) or \
                 (new_key == '6' and self.state == RobotState.RIGHT_TURN):
                # Same key still pressed, repeat state
                print(f"[REPEAT] Continuing {self.state.value}...")
                return
            
            # Check for WASD keys
            if new_key.lower() == required_key.lower():
                # Same key still pressed, repeat the state
                print(f"[REPEAT] Continuing {self.state.value}...")
                return
            elif new_key.lower() == 'w' and self.state != RobotState.FORWARD:
                print(f"\n[STATE] {self.state.value} -> FORWARD")
                self.state = RobotState.FORWARD
                return
            elif new_key.lower() == 's' and self.state != RobotState.BACKWARD:
                print(f"\n[STATE] {self.state.value} -> BACKWARD")
                self.state = RobotState.BACKWARD
                return
            elif new_key.lower() == 'a' and self.state != RobotState.LEFT:
                print(f"\n[STATE] {self.state.value} -> LEFT")
                self.state = RobotState.LEFT
                return
            elif new_key.lower() == 'd' and self.state != RobotState.RIGHT:
                print(f"\n[STATE] {self.state.value} -> RIGHT")
                self.state = RobotState.RIGHT
                return
        
        # No key pressed or different key, return to WAITFORMOVE
        print(f"[STATE] {self.state.value} -> WAITFORMOVE")
        self.state = RobotState.WAITFORMOVE


if __name__ == "__main__":
    debug_mode = False
    
    print("="*60)
    if debug_mode:
        print("UNITREE GO2 INTERACTIVE CONTROL [DEBUG MODE]")
        print("="*60)
        print("\nRunning in DEBUG mode - no robot connection required.")
        print("All commands will be printed to console.")
    else:
        print("UNITREE GO2 INTERACTIVE CONTROL SYSTEM")
        print("="*60)
        print("\nWARNING: Please ensure there are no obstacles around the robot")
        print("           while running this example.")
    
    print("\nPress Enter to continue...")
    input()
    
    if debug_mode:
        # Use mock client in debug mode
        print("\n[DEBUG] Using MockSportClient (no robot connection)")
        sport_client = MockSportClient()
        sport_client.SetTimeout(10.0)
        sport_client.Init()
        print("[DEBUG] Mock client initialized successfully.")
    else:
        # Initialize real robot connection
        if len(sys.argv) > 1 and sys.argv[1] not in ['--debug', '-d']:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0, "enx2887ba59bf0a")

        sport_client = SportClient()  
        sport_client.SetTimeout(10.0)
        sport_client.Init()
        print("\n[INIT] Sport client initialized successfully.")
    
    # Create and run state machine
    state_machine = RobotStateMachine(sport_client, debug_mode=debug_mode)
    state_machine.run()
    
    print("\n[CLEANUP] Shutting down gracefully...")
    print("="*60)

