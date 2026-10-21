#!/usr/bin/env python3
"""Check packet serialization and RX resynchronization using a local pipe."""
import importlib.util
import os
from pathlib import Path
import struct

path = Path(__file__).resolve().parents[1] / 'xuc_fixed_yaw_test.py'
spec = importlib.util.spec_from_file_location('fixed_yaw', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
packet = module.control_packet(20.0, 10.0)
assert len(packet) == 31 and packet[0] == 0xa5 and packet[21] == 0
assert struct.unpack_from('<2f', packet, 1) == (10.0, 20.0)
assert module.crc16(packet[:29]) == int.from_bytes(packet[29:], 'little')
rx = bytearray(struct.pack('<BB6fH', 0x5a, 0x80, 0, 10, 20, 0, 0, 0, 0))
rx[26:] = module.crc16(rx[:26]).to_bytes(2, 'little')
invalid = bytearray(rx)
invalid[-1] ^= 1
read_fd, write_fd = os.pipe()
try:
    os.write(write_fd, b'noise' + invalid + rx)
    receiver = module.Receiver()
    rows = []
    receiver.receive(read_fd, rows.append)
    assert receiver.rx_count == 1 and receiver.crc_fail == 1
    assert receiver.latest == (0x80, 0, 10, 20) and receiver.last_rx > 0
    assert len(rows) == 1
finally:
    os.close(read_fd)
    os.close(write_fd)
print('PASS Python packet/CRC/RX resynchronization; no UART opened')
