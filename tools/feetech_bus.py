"""Minimal Feetech STS (ST3215) serial bus driver on top of pyserial.

Only what the SO-101 tools need: ping, read, write and the EEPROM lock.

Protocol (half duplex, 1 Mbps, multi-byte values little-endian):
  instruction: FF FF ID LEN INSTR PARAM... CHK
  status:      FF FF ID LEN ERR   PARAM... CHK
  LEN = number of params + 2
  CHK = ~(ID + LEN + INSTR/ERR + sum(params)) & 0xFF
"""

import glob
import time
from contextlib import contextmanager

import serial

# Instructions
PING = 0x01
READ = 0x02
WRITE = 0x03

# Control table (STS3215). EEPROM is below address 40, RAM from 40 on.
MIN_POSITION_LIMIT = 9      # u16
MAX_POSITION_LIMIT = 11     # u16
HOMING_OFFSET = 31          # u16, sign-magnitude (bit 11); present = raw - offset
OPERATING_MODE = 33         # u8, 0 = position
TORQUE_ENABLE = 40          # u8
ACCELERATION = 41           # u8, unit 100 step/s^2
GOAL_POSITION = 42          # u16
GOAL_SPEED = 46             # u16, unit step/s
TORQUE_LIMIT = 48           # u16, unit 0.1 %
LOCK = 55                   # u8, 0 = EEPROM writable
PRESENT_POSITION = 56       # u16, sign bit 15
PRESENT_SPEED = 58          # u16, sign bit 15
PRESENT_LOAD = 60           # u16, sign bit 10, unit 0.1 %
PRESENT_VOLTAGE = 62        # u8, unit 0.1 V
PRESENT_TEMPERATURE = 63    # u8, deg C

TICKS_PER_TURN = 4096

# USB vendor 1a86 = WCH, the CH343 on the Waveshare bus servo driver board.
ADAPTER_GLOB = "/dev/serial/by-id/usb-1a86_*"


class FeetechError(RuntimeError):
    pass


def find_port():
    """Return the adapter's stable /dev/serial/by-id path without opening any port."""
    hits = sorted(glob.glob(ADAPTER_GLOB))
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise FeetechError(f"no servo adapter found ({ADAPTER_GLOB}); pass --port")
    raise FeetechError("several WCH adapters found, pass --port:\n  " + "\n  ".join(hits))


def decode_sm(value, sign_bit):
    """Sign-magnitude to int (Feetech stores negatives as magnitude | sign bit)."""
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


def encode_sm(value, sign_bit):
    return (-value) | (1 << sign_bit) if value < 0 else value


class FeetechBus:
    def __init__(self, port, baudrate=1_000_000, timeout=0.05):
        # exclusive=True takes a flock: a second tool (or the ros2_control
        # interface) cannot open the bus at the same time.
        self.ser = serial.Serial(port, baudrate, timeout=0.002, exclusive=True)
        self.port = port
        self.timeout = timeout
        self.last_error = {}  # servo id -> error byte of its last status packet

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- packet level -----------------------------------------------------

    def _transact(self, servo_id, instr, params=b""):
        body = bytes([servo_id, len(params) + 2, instr]) + bytes(params)
        packet = b"\xff\xff" + body + bytes([~sum(body) & 0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        return self._read_status(servo_id, packet)

    def _read_status(self, servo_id, sent):
        buf = bytearray()
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            buf += self.ser.read(self.ser.in_waiting or 1)
            start = buf.find(b"\xff\xff")
            if start < 0 or len(buf) < start + 4:
                continue
            end = start + 4 + buf[start + 3]
            if len(buf) < end:
                continue
            frame = bytes(buf[start:end])
            del buf[:end]
            if frame == sent:  # local echo on some adapters
                continue
            sid, length, err = frame[2], frame[3], frame[4]
            if (~sum(frame[2:-1]) & 0xFF) != frame[-1]:
                raise FeetechError(f"id {servo_id}: bad checksum")
            if sid != servo_id:
                continue
            self.last_error[servo_id] = err
            return frame[5:5 + length - 2]
        raise FeetechError(f"id {servo_id}: no reply")

    # --- register access --------------------------------------------------

    def ping(self, servo_id):
        try:
            self._transact(servo_id, PING)
            return True
        except FeetechError:
            return False

    def read(self, servo_id, addr, size):
        data = self._transact(servo_id, READ, bytes([addr, size]))
        if len(data) != size:
            raise FeetechError(f"id {servo_id}: read {addr} returned {len(data)}/{size} bytes")
        return data

    def write(self, servo_id, addr, data):
        self._transact(servo_id, WRITE, bytes([addr]) + bytes(data))

    def read_u8(self, servo_id, addr):
        return self.read(servo_id, addr, 1)[0]

    def read_u16(self, servo_id, addr):
        return int.from_bytes(self.read(servo_id, addr, 2), "little")

    def write_u8(self, servo_id, addr, value):
        self.write(servo_id, addr, [value & 0xFF])

    def write_u16(self, servo_id, addr, value):
        self.write(servo_id, addr, (value & 0xFFFF).to_bytes(2, "little"))

    @contextmanager
    def eeprom_unlocked(self, servo_id):
        self.write_u8(servo_id, LOCK, 0)
        try:
            yield
        finally:
            self.write_u8(servo_id, LOCK, 1)

    # --- convenience ------------------------------------------------------

    def read_position(self, servo_id):
        return decode_sm(self.read_u16(servo_id, PRESENT_POSITION), 15)

    def read_state(self, servo_id):
        """Position [tick], speed [step/s], load [0.1 %], voltage [V], temperature [C] in one read."""
        d = self.read(servo_id, PRESENT_POSITION, 8)
        return (
            decode_sm(int.from_bytes(d[0:2], "little"), 15),
            decode_sm(int.from_bytes(d[2:4], "little"), 15),
            decode_sm(int.from_bytes(d[4:6], "little"), 10),
            d[6] / 10.0,
            d[7],
        )

    def read_homing_offset(self, servo_id):
        return decode_sm(self.read_u16(servo_id, HOMING_OFFSET), 11)

    def write_homing_offset(self, servo_id, offset):
        self.write_u16(servo_id, HOMING_OFFSET, encode_sm(offset, 11))

    def torque(self, servo_id, enable):
        if enable:
            # Hold where the joint is now instead of jumping to a stale goal.
            self.write_u16(servo_id, GOAL_POSITION, self.read_position(servo_id))
        self.write_u8(servo_id, TORQUE_ENABLE, 1 if enable else 0)
