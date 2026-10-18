"""Bounded vendor position/home tests. --execute moves physical hardware.

Vendor absolute/relative/home commands use about 50 deg/s, not adjustable here.
Bounds are test limits, NOT verified mechanical limits. Requires site clearance.
"""
import argparse
import functools
import json
from pathlib import Path
import statistics
import struct
import threading
import time

from query_ptz_ir import query, read_exact
from test_ptz_small_motion import connect, speed


def command(sock, subcommand, order, yaw=None, pitch=None):
    if subcommand not in (1, 2, 4, 10, 11):
        raise ValueError('Command not allowed')
    body = bytes((15, subcommand, 0))
    if subcommand in (1, 2):
        if yaw is None or pitch is None or abs(yaw) > 30 or abs(pitch) > 15:
            raise ValueError('Command exceeds test bounds')
        body += struct.pack('<ii', round(yaw*100), round(pitch*100))
    frame = b'\xf0\xe0'+struct.pack('<HH', order, len(body))+body
    frame += bytes([functools.reduce(int.__xor__, body, 0)])+b'\xf1\xe1'
    sock.sendall(frame)
    header = read_exact(sock, 6)
    seq, size = struct.unpack('<HH', header[2:])
    if header[:2] != b'\xf0\xe0' or seq != order or not 5 <= size <= 32768:
        raise ValueError('Invalid response header')
    tail = read_exact(sock, size+3)
    if (tail[:3] != body[:3] or tail[3:5] != b'\0\0' or tail[-2:] != b'\xf1\xe1'
            or functools.reduce(int.__xor__, tail[:size], 0) != tail[size]):
        raise ValueError('Invalid response')
    return tail[5:size]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['absolute', 'relative', 'home'])
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        print('Dry run:', args.mode, __doc__)
        return
    events = []
    started = time.monotonic()
    seq = 40000

    def log(kind, **data):
        e = dict(t=round(time.monotonic()-started, 3), kind=kind, **data)
        events.append(e)
        print(json.dumps(e), flush=True)

    with connect() as control, connect() as monitor, connect() as guard:
        def sample(phase):
            nonlocal seq
            seq += 1
            r = query(monitor, 'ptz_status', seq)
            log('angle', phase=phase, data=r['data'])
            if r['data']['connect_status'] != 1 or r['data']['total_status'] & ~5:
                raise RuntimeError('Unexpected connection/activity')
            return r['pitch_deg'], r['yaw_deg']

        def settled(phase):
            values = []
            for _ in range(12):
                values.append(sample(phase))
                time.sleep(.1)
            recent = values[-6:]
            if any(max(v[i] for v in recent)-min(v[i] for v in recent) > .4 for i in (0, 1)):
                raise RuntimeError('Position not stable')
            return tuple(statistics.median(v[i] for v in recent) for i in (0, 1))

        def position(sub, target, phase, index):
            before = settled(phase+'_before')
            max_delta = (8, 25) if sub == 4 else (5, 5)
            errors = []

            def guard_stop():
                try:
                    log('guard_release', response=speed(guard, 0, 0, 42000+index))
                except Exception as exc:
                    errors.append(str(exc))
                    log('guard_error', error=str(exc))

            watchdog = threading.Timer(3.5, guard_stop)
            watchdog.start()
            try:
                if sub == 2:
                    pitch, yaw = target[0]-before[0], target[1]-before[1]
                else:
                    pitch, yaw = target
                result = command(control, sub, 41000+index, yaw=yaw, pitch=pitch)
                log('command', phase=phase, subtype=sub, pitch=pitch, yaw=yaw,
                    response=result.hex(), expected=target)
                values = []
                end = time.monotonic()+2.5
                while time.monotonic() < end:
                    actual = sample(phase)
                    if any(abs(actual[i]-before[i]) > max_delta[i] for i in (0, 1)):
                        raise RuntimeError('Motion exceeded bounded test range')
                    values.append(actual)
                    time.sleep(.06)
            finally:
                try:
                    log('release', response=speed(control, 0, 0, 43000+index))
                    watchdog.cancel()
                finally:
                    watchdog.join()
            if errors:
                raise RuntimeError('Independent stop failed')
            after = settled(phase+'_after')
            error = [round(after[i]-target[i], 3) for i in (0, 1)]
            log('position_result', phase=phase, target=target, actual=after, error=error)
            if any(abs(e) > 1 for e in error):
                raise RuntimeError('Position differs from requested angles by more than 1 degree')
            return after

        try:
            log('initial_release', response=speed(control, 0, 0, 44000))
            initial = settled('baseline')
            if not -2 <= initial[0] <= 12 or not -2 <= initial[1] <= 25:
                raise RuntimeError('Initial position outside reviewed test area')
            log('baseline', initial=initial)
            if args.mode == 'home':
                position(4, (0, 0), 'home', 0)
                position(4, (0, 0), 'home_repeat', 1)
            else:
                sub = 1 if args.mode == 'absolute' else 2
                position(sub, (initial[0]+2, initial[1]), 'pitch_plus2', 0)
                position(sub, initial, 'pitch_return', 1)
                position(sub, (initial[0], initial[1]+2), 'yaw_plus2', 2)
                position(sub, initial, 'yaw_return', 3)
            log('completed', mode=args.mode)
        except BaseException as exc:
            log('aborted', error=str(exc))
            raise
        finally:
            try:
                log('final_release', response=speed(control, 0, 0, 44001))
            finally:
                path = Path(__file__).with_name(args.mode+'_'+time.strftime('%H%M%S')+'.json')
                path.write_text(json.dumps(events, indent=2)+'\n')
                print('Saved:', path)


if __name__ == '__main__':
    main()
