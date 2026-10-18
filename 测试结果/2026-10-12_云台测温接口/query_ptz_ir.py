"""Read-only queries verified against this device's AJ_IR01 binary.

Only status and IR frame statistics are exposed; no motion/set/reset commands.
"""
import argparse
from datetime import datetime
import functools
import json
from pathlib import Path
import socket
import struct

COMMANDS = {'ptz_status': bytes.fromhex('110000'),
            'ir_statistics': bytes.fromhex('0e0112')}


def read_exact(sock, size):
    data = b''
    while len(data) < size:
        part = sock.recv(size - len(data))
        if not part:
            raise EOFError('Device closed connection')
        data += part
    return data


def query(sock, name, order):
    body = COMMANDS[name]
    checksum = functools.reduce(int.__xor__, body, 0)
    packet = b'\xf0\xe0' + struct.pack('<HH', order, len(body)) + body + bytes([checksum]) + b'\xf1\xe1'
    sock.sendall(packet)
    header = read_exact(sock, 6)
    seq, size = struct.unpack('<HH', header[2:])
    if header[:2] != b'\xf0\xe0' or seq != order or not 5 <= size <= 32768:
        raise ValueError('Invalid response header')
    tail = read_exact(sock, size + 3)
    if tail[:3] != body or tail[-2:] != b'\xf1\xe1':
        raise ValueError('Unexpected response command or terminator')
    if functools.reduce(int.__xor__, tail[:size], 0) != tail[size]:
        raise ValueError('Response checksum mismatch')
    status = int.from_bytes(tail[3:5], 'little')
    result = dict(query=name, received_at=datetime.now().astimezone().isoformat(),
                  status=status, frame_hex=(header + tail).hex())
    if status:
        raise ValueError('Device status: ' + str(status))
    if size == 5:
        raise ValueError('Empty response payload')
    result['data'] = json.loads(tail[5:size])
    if name == 'ptz_status':
        result['pitch_deg'] = result['data']['pitch'] / 100
        result['yaw_deg'] = result['data']['yaw'] / 100
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.214')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    results = []
    with socket.create_connection((args.host, 10086), timeout=3) as sock:
        sock.settimeout(3)
        for order, name in enumerate(COMMANDS, 31001):
            result = query(sock, name, order)
            results.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    if args.output:
        args.output.write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
