"""Fixed low-speed PTZ test for this AJ_IR01 version; --execute moves hardware.

Zero speed releases serial control to the original controller; it is not motor-off.
An independent TCP connection releases control after 0.8 seconds per pulse.
This software guard cannot protect against network/power loss.
"""
import argparse
import functools
import json
from pathlib import Path
import socket
import struct
import threading
import time

from query_ptz_ir import query, read_exact


def connect():
    sock = socket.create_connection(('192.168.1.214', 10086), timeout=0.5)
    sock.settimeout(0.5)
    return sock


def speed(sock, yaw, pitch, order):
    assert abs(yaw) <= 1 and abs(pitch) <= 1
    body = bytes.fromhex('0f0000') + struct.pack('bb', yaw, pitch)
    sock.sendall(b'\xf0\xe0' + struct.pack('<HH', order, len(body)) + body
                 + bytes([functools.reduce(int.__xor__, body, 0)]) + b'\xf1\xe1')
    header = read_exact(sock, 6)
    seq, size = struct.unpack('<HH', header[2:])
    if header[:2] != b'\xf0\xe0' or seq != order or not 5 <= size <= 32768:
        raise ValueError('Invalid command response header')
    tail = read_exact(sock, size + 3)
    if (tail[:3] != body[:3] or tail[3:5] != b'\0\0'
            or tail[-2:] != b'\xf1\xe1'
            or functools.reduce(int.__xor__, tail[:size], 0) != tail[size]):
        raise ValueError('Invalid command response')
    return (header + tail).hex()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        print('Dry run: each axis +1 then -1 deg/s, 0.45 seconds per pulse; release and verify between pulses.')
        return
    events = []
    started = time.monotonic()
    seq = 32000

    def log(kind, **data):
        item = dict(t=round(time.monotonic()-started, 3), kind=kind, **data)
        events.append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def sample(sock, phase):
        nonlocal seq
        seq += 1
        r = query(sock, 'ptz_status', seq)
        log('angle', phase=phase, pitch=r['pitch_deg'], yaw=r['yaw_deg'], data=r['data'])
        # SystemStatusJson: bit 0 = gimbal moving, bit 2 = Sony camera state.
        # These are activity flags, not an error code. Abort on other activities.
        if r['data']['connect_status'] != 1 or r['data']['total_status'] & ~5:
            raise RuntimeError('Unhealthy device status')
        return r['pitch_deg'], r['yaw_deg']

    def settle(sock, phase):
        readings = []
        for _ in range(12):
            readings.append(sample(sock, phase))
            time.sleep(0.1)
        if any(max(r[i] for r in readings[-6:])-min(r[i] for r in readings[-6:]) > 0.4 for i in (0, 1)):
            raise RuntimeError('Angle has not settled; aborting remaining motion')
        return readings[-1]

    try:
        with connect() as control, connect() as monitor, connect() as guard:
            try:
                log('release', response=speed(control, 0, 0, 30000))
                initial = settle(monitor, 'baseline')
                for index, (yaw, pitch) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):
                    baseline = sample(monitor, 'before_pulse')
                    guard_errors = []

                    def release_guard():
                        try:
                            log('guard_release', response=speed(guard, 0, 0, 30100+index))
                        except Exception as exc:
                            guard_errors.append(str(exc))
                            log('guard_error', error=str(exc))

                    watchdog = threading.Timer(0.8, release_guard)
                    watchdog.start()
                    pulse_start = time.monotonic()
                    try:
                        log('motion', yaw_speed=yaw, pitch_speed=pitch,
                            response=speed(control, yaw, pitch, 30200+index))
                        while time.monotonic()-pulse_start < 0.45:
                            current = sample(monitor, 'pulse_'+str(index))
                            if any(abs(current[i]-baseline[i]) > 2 for i in (0, 1)):
                                raise RuntimeError('Movement exceeded 2 degree test bound')
                            time.sleep(0.05)
                    finally:
                        try:
                            log('release', response=speed(control, 0, 0, 30300+index))
                        finally:
                            watchdog.join()
                    if guard_errors:
                        raise RuntimeError('Independent release failed')
                    after = settle(monitor, 'after_pulse_'+str(index))
                    log('pulse_result', index=index, pitch_change=round(after[0]-baseline[0], 3),
                        yaw_change=round(after[1]-baseline[1], 3))
                    axis = 1 if yaw else 0
                    delta = after[axis]-baseline[axis]
                    if not 0.08 <= abs(delta) <= 2:
                        raise RuntimeError('Expected small axis motion not confirmed')
                log('completed', initial=initial, final=after)
            finally:
                log('final_release', response=speed(control, 0, 0, 30999))
    except BaseException as exc:
        log('aborted', error=str(exc))
        raise
    finally:
        output = Path(__file__).with_name('motion_test_'+time.strftime('%H%M%S')+'.json')
        output.write_text(json.dumps(events, ensure_ascii=False, indent=2)+'\n')
        print('Saved:', output)


if __name__ == '__main__':
    main()
