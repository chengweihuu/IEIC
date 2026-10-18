"""Low-speed feedback positioning/zero and client-disconnect test.

Only sends speed commands of magnitude 1 deg/s. Does not use vendor positioning
or home commands. Zero is the reported angle zero, not mechanical homing.
Software safeguards do not protect against total network loss.
"""
import argparse
import json
from pathlib import Path
import socket
import statistics
import threading
import time

from query_ptz_ir import query
from test_ptz_small_motion import connect, speed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['position', 'disconnect', 'zero'])
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        print('Dry run:', args.mode, __doc__)
        return
    events = []
    start = time.monotonic()
    seq = 46000
    cmd_seq = 48000

    def log(kind, **data):
        e = dict(t=round(time.monotonic()-start, 3), kind=kind, **data)
        events.append(e)
        print(json.dumps(e), flush=True)

    with connect() as control, connect() as monitor, connect() as guard:
        def send(sock, yaw=0, pitch=0):
            nonlocal cmd_seq
            cmd_seq += 1
            reply = speed(sock, yaw, pitch, cmd_seq)
            log('command', yaw_speed=yaw, pitch_speed=pitch, response=reply)

        def sample(phase):
            nonlocal seq
            seq += 1
            r = query(monitor, 'ptz_status', seq)
            log('angle', phase=phase, data=r['data'])
            if r['data']['connect_status'] != 1 or r['data']['total_status'] & ~5:
                raise RuntimeError('Unexpected device connection/activity')
            p, y = r['pitch_deg'], r['yaw_deg']
            if not -3 <= p <= 15 or not -5 <= y <= 27:
                raise RuntimeError('Outside reviewed test bounds')
            return p, y

        def settle(phase):
            values = []
            for _ in range(12):
                values.append(sample(phase))
                time.sleep(.1)
            recent = values[-6:]
            if any(max(v[i] for v in recent)-min(v[i] for v in recent) > .4 for i in (0, 1)):
                raise RuntimeError('Position not stable')
            return tuple(statistics.median(v[i] for v in recent) for i in (0, 1))

        def guarded(action, timeout):
            guard_errors = []
            fired = threading.Event()

            def guard_stop():
                fired.set()
                try:
                    # Separate sequence range avoids overlap with the control socket.
                    log('guard_release', response=speed(guard, 0, 0, 49000))
                except Exception as exc:
                    guard_errors.append(str(exc))
                    log('guard_error', error=str(exc))

            timer = threading.Timer(timeout, guard_stop)
            timer.start()
            try:
                action()
            finally:
                try:
                    send(control)
                    timer.cancel()
                finally:
                    timer.join()
            if fired.is_set() or guard_errors:
                raise RuntimeError('Independent watchdog intervened')

        def move_axis(axis, target, phase):
            before = settle(phase+'_before')
            delta = target-before[axis]
            if abs(delta) <= .3:
                log('already_at_target', phase=phase, target=target, actual=before)
                return before
            direction = 1 if delta > 0 else -1
            duration = min(29, abs(delta)*1.15+2)

            def action():
                send(control, yaw=direction if axis == 1 else 0,
                     pitch=direction if axis == 0 else 0)
                end = time.monotonic()+duration
                history = []
                while time.monotonic() < end:
                    actual = sample(phase)
                    now = time.monotonic()
                    if abs(actual[1-axis]-before[1-axis]) > 1.2:
                        raise RuntimeError('Unexpected other-axis motion')
                    if direction*(actual[axis]-before[axis]) < -.6:
                        raise RuntimeError('Unexpected direction')
                    if abs(actual[axis]-before[axis]) > abs(delta)+1:
                        raise RuntimeError('Overshoot bound exceeded')
                    history.append((now, actual[axis]))
                    recent = [(t,v) for t,v in history if t >= now-2]
                    if now-history[0][0] > 3 and direction*(actual[axis]-recent[0][1]) < .4:
                        raise RuntimeError('Motion stalled')
                    if direction*(target-actual[axis]) <= .2:
                        return
                    time.sleep(.08)
                raise RuntimeError('Target timeout')

            guarded(action, duration+.8)
            after = settle(phase+'_after')
            error = round(after[axis]-target, 3)
            log('position_result', phase=phase, axis=axis, target=target, actual=after, error=error)
            if abs(error) > .6:
                raise RuntimeError('Position outside test tolerance')
            return after

        try:
            send(control)
            initial = settle('baseline')
            log('baseline', initial=initial)
            if args.mode == 'position':
                for axis in (0, 1):
                    move_axis(axis, initial[axis]+3, 'absolute_plus3_'+str(axis))
                    move_axis(axis, initial[axis], 'absolute_return_'+str(axis))
                    current = settle('relative_baseline')
                    move_axis(axis, current[axis]+2, 'relative_plus2_'+str(axis))
                    move_axis(axis, initial[axis], 'relative_return_'+str(axis))
            elif args.mode == 'zero':
                move_axis(1, 0, 'yaw_software_zero')
                move_axis(0, 0, 'pitch_software_zero')
            else:
                samples = []

                def disconnect_action():
                    with connect() as client:
                        send(client, yaw=1)
                        time.sleep(.35)
                        before_close = sample('before_disconnect')
                        client.shutdown(socket.SHUT_RDWR)
                    log('client_closed', angle=before_close)
                    end = time.monotonic()+1.4
                    while time.monotonic() < end:
                        actual = sample('after_disconnect_before_guard_stop')
                        samples.append(actual)
                        if abs(actual[1]-initial[1]) > 2.5 or abs(actual[0]-initial[0]) > 1:
                            raise RuntimeError('Disconnect movement exceeded bound')
                        time.sleep(.1)

                guarded(disconnect_action, 2.5)
                after = settle('after_explicit_stop')
                log('disconnect_result', observation_seconds=1.4,
                    samples_before_explicit_stop=samples, after_explicit_stop=after)
                move_axis(1, initial[1], 'disconnect_return')
            final = settle('final')
            log('completed', mode=args.mode, initial=initial, final=final)
        except BaseException as exc:
            log('aborted', error=str(exc))
            raise
        finally:
            try:
                send(control)
            finally:
                path = Path(__file__).with_name('slow_'+args.mode+'_'+time.strftime('%H%M%S')+'.json')
                path.write_text(json.dumps(events, indent=2)+'\n')
                print('Saved:', path, flush=True)


if __name__ == '__main__':
    main()
