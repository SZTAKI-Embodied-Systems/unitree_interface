import sys
import os
import threading
import termios
import tty
import select
import atexit

from unitree_interface.unitreeinterface import UnitreeInterface

class TerminalHandler:
    def __init__(self, robot_interface: UnitreeInterface):
        self._fd = sys.stdin.fileno()
        self._orig_termios = termios.tcgetattr(self._fd)
        self.robot_interface = robot_interface

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
                        print("\nEMERGENCY STOP")

                    elif ch == '\x03':            # Ctrl+C   NOT WORKING AS EXPECTED
                        print('\n Ctrl+C pressed, exiting.')
                        
        except KeyboardInterrupt:
            print("Terminal handler exiting.")

        finally:
            self.restore_terminal()

    def restore_terminal(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._orig_termios)

    def close(self):
        self._thread.join(timeout=0.2)
        self.restore_terminal()