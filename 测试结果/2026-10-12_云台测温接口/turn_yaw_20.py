"""Fixed +20 degree yaw test at 1 degree/s; stops at new position. Requires --execute."""
import argparse
import json
from pathlib import Path
import statistics
import threading
import time
from query_ptz_ir import query
from test_ptz_small_motion import connect, speed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    if not parser.parse_args().execute:
        print(__doc__)
        return
    events = []
    seq = 36000
    start = time.monotonic()

    def log(kind, **data):
        event = dict(t=round(time.monotonic()-start, 3), kind=kind, **data)
        events.append(event)
        print(json.dumps(event), flush=True)

    with connect() as control, connect() as monitor, connect() as guard:
        def sample():
            nonlocal seq
            seq += 1
            r = query(monitor, 'ptz_status', seq)
            log('angle', data=r['data'])
            if r['data']['connect_status'] != 1 or r['data']['total_status'] & ~5:
                raise RuntimeError('Unexpected device connection/activity')
            return r['pitch_deg'], r['yaw_deg']

        def settled():
            readings = []
            for _ in range(15):
                readings.append(sample())
                time.sleep(.1)
            recent = readings[-6:]
            if any(max(r[i] for r in recent)-min(r[i] for r in recent) > .4 for i in (0, 1)):
                raise RuntimeError('Position not stable')
            return tuple(statistics.median(r[i] for r in recent) for i in (0, 1))

        guard_errors = []
        def guard_stop():
            try:
                log('guard_release', response=speed(guard, 0, 0, 37001))
            except Exception as exc:
                guard_errors.append(str(exc))
                log('guard_error', error=str(exc))

        watchdog = None
        try:
            log('release', response=speed(control, 0, 0, 37002))
            initial = settled()
            if not -150 < initial[1] < 150:
                raise RuntimeError('Initial yaw outside conservative test range')
            log('baseline', pitch=initial[0], yaw=initial[1], target_yaw=initial[1]+20)
            watchdog = threading.Timer(24, guard_stop)
            watchdog.start()
            moving_since = time.monotonic()
            progress = []
            try:
                log('start_motion', response=speed(control, 1, 0, 37003))
                while time.monotonic()-moving_since < 23:
                    pitch, yaw = sample()
                    elapsed = time.monotonic()-moving_since
                    delta = yaw-initial[1]
                    if not -1 < delta < 21.5 or abs(pitch-initial[0]) > 1.5:
                        raise RuntimeError('Angle bound exceeded')
                    progress.append((elapsed, yaw))
                    recent = [(t,y) for t,y in progress if t >= elapsed-2]
                    if elapsed > 3 and yaw-recent[0][1] < .4:
                        raise RuntimeError('Insufficient movement; possible obstruction or control conflict')
                    if delta >= 19.8:
                        break
                    time.sleep(.1)
                else:
                    raise RuntimeError('Motion time limit exceeded')
            finally:
                log('release', response=speed(control, 0, 0, 37004))
                watchdog.cancel()
                watchdog.join()
            if guard_errors:
                raise RuntimeError('Independent stop failed')
            final = settled()
            log('completed', initial=initial, final=final, yaw_change=round(final[1]-initial[1], 3))
        except BaseException as exc:
            log('aborted', error=str(exc))
            raise
        finally:
            try:
                log('final_release', response=speed(control, 0, 0, 37005))
                if watchdog:
                    watchdog.cancel()
            finally:
                if watchdog:
                    watchdog.join()
                p = Path(__file__).with_name('yaw20_'+time.strftime('%H%M%S')+'.json')
                p.write_text(json.dumps(events, indent=2)+'\n')
                print('Saved:', p, flush=True)


if __name__ == '__main__':
    main()
