"""One writer per radio: open a serial port and set TIOCEXCL, so the kernel refuses every later open
of that port (EBUSY) while it is held. pyserial's exclusive=True is only an advisory flock that a
second process opening the port does not have to honour (combined-stack design, critic 2026-09-29)."""
import fcntl
import termios

import serial


def open_exclusive(port, baudrate=921600, timeout=0.1):
    ser = serial.Serial(port, baudrate, timeout=timeout)
    try:
        fcntl.ioctl(ser.fileno(), termios.TIOCEXCL)
    except OSError:
        ser.close()
        raise
    return ser
