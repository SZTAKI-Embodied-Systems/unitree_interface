import sys
import os
import threading
import termios
import tty
import select
import atexit
import time
import _thread

from unitree_interface.unitreeinterface import UnitreeInterface

class TerminalHandler:
    def __init__(self, robot_interface: UnitreeInterface):
        self._fd = sys.stdin.fileno()
        self._orig_termios = termios.tcgetattr(self._fd)
        self.robot_interface = robot_interface
        self.interrupt_main_sent = False

        atexit.register(self.restore_terminal)

        self._thread = threading.Thread(target=self._run, daemon=False)
        self._thread.start()
        print(f"[TerminalHandler] Started. Press 'q' for emergency stop, Ctrl+C to exit.")

    def _run(self):
        tty.setcbreak(self._fd)
        try:
            while True:
                r, w, e = select.select([self._fd], [], [], 0.1)
                if r:
                    ch = os.read(self._fd, 1).decode()
                    if ch in ('q', 'Q'):
                        self.robot_interface.emergency_event.set()
                        print("\n[TerminalHandler] EMERGENCY STOP")
                        if not self.interrupt_main_sent:
                            self.interrupt_main_sent = True
                            time.sleep(1.0)
                            _thread.interrupt_main() # Raise Interrupt in main thread to exit program
                        
                    elif ch in ('s', 'S'):
                        print('\n[TerminalHandler] Stop pressed, closing connection and exiting.')
                        self.robot_interface.CloseConnection()
                        if not self.interrupt_main_sent:
                            self.interrupt_main_sent = True
                            time.sleep(1.0)
                            _thread.interrupt_main() # Raise Interrupt in main thread to exit program
                        
        except KeyboardInterrupt:
            print("[TerminalHandler] Terminal handler exiting.")

        finally:
            self.restore_terminal()

    def restore_terminal(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._orig_termios)

    def close(self):
        self._thread.join(timeout=0.2)
        print("[TerminalHandler] Terminal handler closed.")
        self.restore_terminal()