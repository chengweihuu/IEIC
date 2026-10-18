"""Test modest zoom and manual/one-shot focus, restoring original settings.

--execute changes lens settings. No gimbal movement commands are sent.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import time
from camera_protocol import connect, request, sony_status


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    if not p.parse_args().execute:
        print(__doc__)
        return
    folder = Path(__file__).with_name('lens_'+time.strftime('%H%M%S'))
    folder.mkdir()
    events = []
    def record(phase, **data):
        e = dict(phase=phase, time=time.time(), **data)
        events.append(e)
        (folder/'results.json').write_text(json.dumps(events, indent=2))
        print(json.dumps(e), flush=True)
    def shot(name):
        with (folder/(name+'.log')).open('w') as err:
            subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-rtsp_transport','tcp',
                '-i','rtsp://192.168.1.214:8554/visible','-frames:v','1','-update','1','-y',
                str(folder/(name+'.jpg'))], stderr=err, stdout=subprocess.DEVNULL,
                timeout=15, check=True)
    with connect() as s:
        initial = sony_status(s)
        record('initial', **initial)
        if not initial['magnification'] == 10 or initial['focus_mode'] not in (0,1):
            raise RuntimeError('Unexpected baseline; stop before lens changes')
        try:
            shot('before')
            request(s,2,8,struct.pack('<h',20))
            time.sleep(3)
            record('zoom_2x', **sony_status(s))
            shot('zoom_2x')
            request(s,2,2,b'\x01')
            request(s,2,9,struct.pack('<i',max(4096, initial['focus_position']-4096)))
            time.sleep(2)
            record('manual_focus', **sony_status(s))
            shot('manual_focus')
            request(s,2,4)
            time.sleep(4)
            record('one_shot_af', **sony_status(s))
            shot('one_shot_af')
        finally:
            # Try every restoration step even if an earlier one fails.
            errors = []
            for cmd, data in [(0,b'\0\0'), (3,b'\0\0'),
                              (8,struct.pack('<h',initial['magnification'])),
                              (9,struct.pack('<i',initial['focus_position'])),
                              (2,bytes([initial['focus_mode']]))]:
                try: request(s,2,cmd,data)
                except Exception as exc: errors.append(str(exc))
            time.sleep(3)
            record('restored', errors=errors, **sony_status(s))
            shot('restored')
            if errors: raise RuntimeError('Restoration errors: '+str(errors))
    print('Saved:',folder)


if __name__ == '__main__':
    main()
