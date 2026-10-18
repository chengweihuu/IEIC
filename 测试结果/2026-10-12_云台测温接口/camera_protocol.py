"""AJ_IR01 camera framing helpers. No gimbal commands are exposed."""
import functools
import socket
import struct
from query_ptz_ir import read_exact


def connect():
    return socket.create_connection(('192.168.1.214', 10086), timeout=5)


def request(sock, group, command, payload=b'', order=50000):
    if group not in (1, 2):
        raise ValueError('Camera commands only')
    return _exchange(sock, bytes((14, group, command))+payload, order)


def capture_ir(sock, order=51000):
    """Capture only IR still image/data (mask 2, video duration 0), no motion."""
    return _exchange(sock, bytes.fromhex('050000')+struct.pack('<ii', 2, 0), order)


def _exchange(sock, body, order):
    frame = b'\xf0\xe0'+struct.pack('<HH', order, len(body))+body
    frame += bytes([functools.reduce(int.__xor__, body, 0)])+b'\xf1\xe1'
    sock.sendall(frame)
    header = read_exact(sock, 6)
    seq, size = struct.unpack('<HH', header[2:])
    if header[:2] != b'\xf0\xe0' or seq != order or not 5 <= size <= 65535:
        raise ValueError('Invalid response header')
    tail = read_exact(sock, size+3)
    if tail[:3] != body[:3] or tail[-2:] != b'\xf1\xe1':
        raise ValueError('Invalid echoed command or tail')
    if functools.reduce(int.__xor__, tail[:size], 0) != tail[size]:
        raise ValueError('Response checksum failed')
    status = int.from_bytes(tail[3:5], 'little')
    if status:
        raise RuntimeError('Camera status '+str(status))
    return tail[5:size]


def sony_status(sock):
    result = {}
    for cmd, name, fmt in [(1, 'focus_mode', '<B'), (6, 'zoom_position', '<H'),
                           (7, 'magnification', '<h'), (10, 'focus_position', '<i')]:
        result[name] = struct.unpack(fmt, request(sock, 2, cmd, order=50000+cmd))[0]
    return result


if __name__ == '__main__':
    import json
    with connect() as sock:
        print(json.dumps(sony_status(sock)))
