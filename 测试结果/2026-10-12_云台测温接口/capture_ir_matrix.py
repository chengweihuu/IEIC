"""Capture an IR temperature matrix, download via SSH, and export Celsius CSV.

Default captures a still on the camera host; --fir decodes a local file only.
No gimbal movement. Python standard library and scp required for capture.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path, PurePosixPath
import re
import struct
import subprocess
from camera_protocol import connect, capture_ir


def decode(path):
    data = path.read_bytes()
    if len(data) < 64 or data[:4] != b'FIR\0':
        raise ValueError('Not an AJ_IR01 FIR file')
    height, width = struct.unpack_from('<HH', data, 24)
    if (width, height) != (640, 512) or len(data) != 64+width*height*2:
        raise ValueError('Unexpected dimensions or payload size')
    header_max, header_min = struct.unpack_from('<ff', data, 28)
    values = struct.unpack_from('<'+str(width*height)+'h', data, 64)
    # Vendor saveFirFile writes signed int16, truncating Celsius * 10.
    # Its rows are already flipped relative to the SDK buffer: preserve FIR order.
    if not all(math.isfinite(t) for t in (header_max, header_min)):
        raise ValueError('Non-finite header temperatures')
    if abs(max(values)/10-header_max) > 0.11 or abs(min(values)/10-header_min) > 0.11:
        raise ValueError('Header and matrix temperature extrema disagree')
    output = path.with_suffix('.csv')
    with output.open('w', newline='') as f:
        writer = csv.writer(f)
        for y in range(height):
            writer.writerow(format(v/10, '.1f') for v in values[y*width:(y+1)*width])
    result = dict(source=str(path), csv=str(output), width=width, height=height,
                  pixels=len(values), unit='degC', storage_step_degC=0.1,
                  min_degC=min(values)/10, max_degC=max(values)/10,
                  mean_degC=sum(values)/len(values)/10,
                  header_min_degC=header_min, header_max_degC=header_max,
                  row_order='FIR file order; SDK rows reversed by vendor exporter',
                  image_alignment_verified=False, measurement_accuracy_verified=False)
    path.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fir', type=Path, help='Decode existing file without connecting')
    args = p.parse_args()
    if args.fir:
        print(json.dumps(decode(args.fir), ensure_ascii=False, indent=2))
        return
    folder = Path(__file__).with_name('红外矩阵')/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    with connect() as s:
        s.settimeout(30)
        reply = capture_ir(s)
    if len(reply) < 5 or struct.unpack_from('<I', reply)[0] != 2:
        raise ValueError('Unexpected capture response')
    files = json.loads(reply[4:])
    (folder/'capture_reply.json').write_text(json.dumps(files, indent=2))
    remote = []
    for key, ext in [('ir_fir_path','fir'), ('ir_ddt_path','ddt'), ('ir_pic_path','jpg')]:
        path = str(PurePosixPath(files[key]))
        if not re.fullmatch(r'/mnt/sdcard/data/[A-Za-z0-9_-]+/[A-Za-z0-9_-]+\.'+ext, path):
            raise ValueError('Unexpected camera export path: '+path)
        remote.append('root@192.168.1.214:'+path)
    print('Downloading capture; enter camera host SSH password when prompted.', flush=True)
    subprocess.run(['scp', '-O', *remote, str(folder)], check=True)
    print(json.dumps(decode(folder/PurePosixPath(files['ir_fir_path']).name),
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
