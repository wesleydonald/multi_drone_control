"""One writer per radio: a port held by open_exclusive refuses a second open (TIOCEXCL), as a second
elrs_interface or `cat /dev/QUAD4` would try while the stack is up."""
import os
import pty
import sys

import pytest
import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from drone_communication.serial_lock import open_exclusive  # noqa: E402


def test_a_held_port_refuses_a_second_open():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    held = open_exclusive(path, baudrate=921600)
    try:
        with pytest.raises((serial.SerialException, OSError)):
            serial.Serial(path, 921600, timeout=0.1)
    finally:
        held.close()
        os.close(master)
        os.close(slave)
    again = serial.Serial(path, 921600, timeout=0.1) if os.path.exists(path) else None
    if again is not None:
        again.close()
