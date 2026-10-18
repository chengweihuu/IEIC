"""One horizontal ~5 degree move, 3 second pause, then return. --execute moves hardware."""
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
    started = time.monotonic()
    seq = 34000

    def log(kind, **data):
        item = dict(t=round(time.monotonic()-started, 3), kind=kind, **data)
        events.append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)

    with connect() as control, connect() as monitor, connect() as guard:
        def sample(phase):
            nonlocal seq
            seq += 1
            result = query(monitor, 'ptz_status', seq)
            log('angle', phase=phase, data=result['data'])
            if result['data']['connect_status'] != 1 or result['data']['total_status'] & ~5:
                raise RuntimeError('Unexpected connection/activity status')
            return result['pitch_deg'], result['yaw_deg']

        def stationary(phase, seconds):
            readings = []
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                readings.append(sample(phase))
                time.sleep(.1)
            recent = readings[-6:]
            if any(max(r[i] for r in recent)-min(r[i] for r in recent) > .4 for i in (0, 1)):
                raise RuntimeError('Angles did not settle')
            return tuple(statistics.median(r[i] for r in recent) for i in (0, 1))

        def move(direction, target, index):
            errors = []

            def timeout_stop():
                try:
                    log('watchdog_release', response=speed(guard, 0, 0, 35000+index))
                except Exception as exc:
                    errors.append(str(exc))

            watchdog = threading.Timer(7, timeout_stop)
            watchdog.start()
            deadline = time.monotonic()+6.5
            try:
                log('start_motion', direction=direction, target=target,
                    response=speed(control, direction, 0, 35100+index))
                while time.monotonic() < deadline:
                    pitch, yaw = sample('outward' if index == 0 else 'return')
                    if abs(pitch-initial[0]) > 1.5 or abs(yaw-initial[1]) > 6.5:
                        raise RuntimeError('Movement exceeded bounds')
                    if direction*(target-yaw) <= .2:
                        break
                    time.sleep(.08)
                else:
                    raise RuntimeError('Target not reached within time bound')
            finally:
                try:
                    log('release', response=speed(control, 0, 0, 35200+index))
                    watchdog.cancel()
                finally:
                    watchdog.join()
            if errors:
                raise RuntimeError('Watchdog release failed: '+str(errors))

        try:
            log('initial_release', response=speed(control, 0, 0, 35300))
            initial = stationary('baseline', 1.2)
            log('baseline', pitch=initial[0], yaw=initial[1])
            move(1, initial[1]+5, 0)
            paused = stationary('pause_3_seconds', 3)
            log('outward_result', yaw_change=round(paused[1]-initial[1], 3))
            if not 4 <= paused[1]-initial[1] <= 6:
                raise RuntimeError('Unexpected outward movement')
            move(-1, initial[1], 1)
            final = stationary('final', 1.5)
            if abs(final[1]-initial[1]) > .6:
                raise RuntimeError('Return position outside tolerance')
            log('completed', initial=initial, paused=paused, final=final,
                return_error=round(final[1]-initial[1], 3))
        except BaseException as exc:
            log('aborted', error=str(exc))
            raise
        finally:
            try:
                log('final_release', response=speed(control, 0, 0, 35301))
            finally:
                output = Path(__file__).with_name('visible_motion_'+time.strftime('%H%M%S')+'.json')
                output.write_text(json.dumps(events, ensure_ascii=False, indent=2)+'\n')
                print('Saved:', output, flush=True)


if __name__ == '__main__':
    main()
