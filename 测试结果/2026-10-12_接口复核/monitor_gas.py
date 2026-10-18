#!/usr/bin/env python3
"""Read existing gas logs over SSH, display software values and save CSV.

No sensor commands, remote writes or stored passwords. Units remain unverified.
Run with --replay FILE to process a saved log without connecting to the robot.
"""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import queue
import re
import shlex
import subprocess
import sys
import threading
import time

from decode_gas_log import decode_reply

FIELDS = ['received_at', 'source_log_time', 'channel', 'value', 'raw_value',
          'gas_power', 'error_code', 'crc_ok', 'status', 'unit', 'packet_hex']
NAMES = {3: ['温度', '湿度'], 5: ['氧气'], 10: ['CO']}


def now():
    return datetime.now().astimezone().isoformat(timespec='milliseconds')


def parse_line(line, received_at):
    if 'PtlGasSensor_RK_GAS01.cpp' not in line:
        return []
    match = re.search(r'RECV\s*\[([0-9A-Fa-f]+)\]', line)
    if not match:
        return []
    try:
        result = decode_reply(match[1])
    except ValueError as exc:
        return [dict(received_at=received_at, source_log_time=line[:19],
                     status='decode_error: ' + str(exc), packet_hex=match[1])]
    values = result['software_values']
    status = ('crc_failed' if not result['crc_ok'] else
              'device_error_' + str(result['error_code']) if result['error_code'] else
              'unsupported' if values is None else 'protocol_ok_units_unverified')
    rows = []
    names = NAMES.get(result['channel'], [str(result['channel'])])
    for i, name in enumerate(names):
        rows.append(dict(received_at=received_at, source_log_time=line[:19],
                         channel=name, value=values[i] if values is not None else '',
                         raw_value=result.get('raw_values', [''] * len(names))[i],
                         gas_power=result['gas_power'], error_code=result['error_code'],
                         crc_ok=result['crc_ok'], status=status, unit='unverified',
                         packet_hex=result['packet_hex']))
    return rows


def reader(stream, events, kind):
    try:
        for line in stream:
            events.put((kind, line.rstrip('\n'), now()))
    finally:
        events.put((kind + '_end', '', now()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.120')
    parser.add_argument('--user', default='robot')
    parser.add_argument('--remote-log', default='/home/root/log/other.log')
    parser.add_argument('--output', type=Path, help='New directory for this recording')
    parser.add_argument('--duration', type=float, default=0, help='Stop after N seconds; 0 means until Ctrl+C')
    parser.add_argument('--replay', type=Path, help='Offline log replay')
    args = parser.parse_args(argv)
    if args.duration < 0:
        parser.error('--duration must be nonnegative')
    if args.host.startswith('-') or args.user.startswith('-'):
        parser.error('Invalid SSH destination')
    output = args.output or Path('测试结果/气体监测') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    print(f'记录目录：{output.resolve()}', flush=True)
    print('显示的是软件值，单位未确认；历史日志可能约一分钟才输出一次，不能视为实时采样。', flush=True)
    events = queue.Queue()
    proc = None
    source = None
    count = 0
    result_code = 0
    started = time.monotonic()
    latest = {}
    last_notice = started
    try:
        with (output / 'readings.csv').open('x', newline='', encoding='utf-8-sig') as csv_file, \
             (output / 'events.csv').open('x', newline='', encoding='utf-8-sig') as event_file, \
             (output / 'gas_raw.log').open('x', encoding='utf-8') as raw_file:
            readings = csv.DictWriter(csv_file, fieldnames=FIELDS)
            readings.writeheader()
            markers = csv.writer(event_file)
            markers.writerow(['recorded_at', 'event'])
            markers.writerow([now(), 'offline_replay_start' if args.replay else 'monitor_start'])
            csv_file.flush(); event_file.flush()
            if args.replay:
                source = args.replay.open(encoding='utf-8', errors='replace')
            else:
                command = ['ssh', '-T', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15',
                           '-o', 'ServerAliveCountMax=2', args.user + '@' + args.host,
                           'tail -n 0 -F -- ' + shlex.quote(args.remote_log)]
                proc = subprocess.Popen(command, stdout=subprocess.PIPE, text=True,
                                        encoding='utf-8', errors='replace')
                source = proc.stdout
                # SSH obtains the password from /dev/tty. Start marker input after first reply.
            threading.Thread(target=reader, args=(source, events, 'log'), daemon=True).start()
            marker_started = False
            while True:
                elapsed = time.monotonic() - started
                if args.duration and elapsed >= args.duration:
                    markers.writerow([now(), 'duration_limit'])
                    break
                try:
                    kind, line, received = events.get(timeout=1)
                except queue.Empty:
                    kind, line, received = '', '', now()
                if kind == 'log_end':
                    if proc:
                        try:
                            result_code = proc.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            result_code = 1
                        print(f'SSH读取结束（退出码{result_code}），记录已保存。', flush=True)
                    break
                if kind == 'marker':
                    if line.strip():
                        markers.writerow([received, line.strip()]); event_file.flush()
                        print(f'已标记：{line.strip()}', flush=True)
                if kind == 'log':
                    if 'PtlGasSensor_RK_GAS01.cpp' in line:
                        raw_file.write(line + '\n'); raw_file.flush()
                    rows = parse_line(line, received)
                    for row in rows:
                        readings.writerow(row); count += 1
                        if row.get('value') != '' and row.get('status') == 'protocol_ok_units_unverified':
                            latest[row['channel']] = (row['value'], time.monotonic())
                        print(f"[{row['source_log_time']}] {row.get('channel', '?')} 软件值={row.get('value', '')} "
                              f"原始值={row.get('raw_value', '')} 状态={row['status']}", flush=True)
                    if rows:
                        csv_file.flush()
                        if not args.replay and not marker_started and sys.stdin.isatty():
                            print('可输入“呼气开始”“呼气结束”等文字并回车，记录实验时刻。Ctrl+C停止。', flush=True)
                            threading.Thread(target=reader, args=(sys.stdin, events, 'marker'), daemon=True).start()
                            marker_started = True
                if not args.replay and time.monotonic() - last_notice >= 15:
                    status = '；'.join(f'{name}={value}（{time.monotonic()-seen:.0f}秒前）'
                                      for name, (value, seen) in latest.items())
                    print('最近成功回复：' + (status or '尚未收到，正在等待日志；不是读数0。'), flush=True)
                    last_notice = time.monotonic()
    except KeyboardInterrupt:
        print('\n已停止，已接收记录保存在CSV中。', flush=True)
    finally:
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
        if source:
            source.close()
        (output / 'session.json').write_text(json.dumps(dict(
            ended_at=now(), mode='replay' if args.replay else 'ssh_log_follow',
            rows=count, ssh_host=args.host, remote_log=args.remote_log,
            units='unverified', timestamps='Local receive time and remote log time are NOT synchronized',
            exit_code=result_code), ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'共保存{count}行：{output / "readings.csv"}', flush=True)
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())
