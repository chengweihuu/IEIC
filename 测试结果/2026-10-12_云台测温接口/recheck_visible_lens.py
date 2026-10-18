"""Repeat lens check with settling time; --execute changes the lens and restores it."""
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
        return
    folder = Path(__file__).with_name('lens_recheck_'+time.strftime('%H%M%S'))
    folder.mkdir()
    events = []
    with connect() as s:
        initial = sony_status(s)
        if initial['magnification'] != 10 or initial['focus_mode'] != 0:
            raise RuntimeError('Expected 1x autofocus baseline')
        def save(name):
            row = dict(phase=name, time=time.time(), **sony_status(s))
            events.append(row)
            (folder/'results.json').write_text(json.dumps(events, indent=2))
            print(row, flush=True)
            subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-rtsp_transport','tcp',
                '-i','rtsp://192.168.1.214:8554/visible','-frames:v','1','-update','1','-y',
                str(folder/(name+'.jpg'))], timeout=20, check=True)
        try:
            save('before')
            request(s,2,8,struct.pack('<h',20))
            time.sleep(15)
            save('zoom_2x_wait15')
            request(s,2,2,b'\x01')
            request(s,2,9,struct.pack('<i',24576))
            time.sleep(8)
            save('focus_24576')
            request(s,2,8,struct.pack('<h',10))
            request(s,2,2,b'\x00')
            time.sleep(8)
            save('wide_auto')
            request(s,2,2,b'\x01')
            request(s,2,9,struct.pack('<i',4096))
            time.sleep(5)
            save('wide_manual_4096')
            request(s,2,4)
            time.sleep(10)
            save('wide_oneshot')
        finally:
            errors=[]
            for cmd,data in [(0,b'\0\0'),(3,b'\0\0'),(8,struct.pack('<h',10)),
                             (9,struct.pack('<i',initial['focus_position'])),(2,b'\0')]:
                try: request(s,2,cmd,data)
                except Exception as exc: errors.append(str(exc))
            time.sleep(5)
            save('restored')
            if errors: raise RuntimeError(str(errors))


if __name__ == '__main__':
    main()
