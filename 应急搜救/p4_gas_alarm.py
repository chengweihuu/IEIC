"""P4 sensor reading -> configured threshold -> observation location -> warning.

Python 3.8+, standard library only. No robot motion or sensor control commands.
P4配置.json selects CO for temporary integration; no VOC substitution is implied.
VOC uses a caller-provided reader/sample until its device interface is confirmed.
The local RK_GAS01 log adapter supports only observed CO/O2/temperature/humidity
software values. These are not calibrated concentration measurements.
"""
import argparse
import binascii
import copy
import csv
import ctypes as c
import ctypes.util
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
import wave


def _now():
    return datetime.now().astimezone().isoformat(timespec='milliseconds')


def _epoch(value):
    if not isinstance(value, str):
        raise ValueError('observed_at must be an ISO timestamp with timezone')
    stamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if stamp.tzinfo is None or stamp.utcoffset() is None:
        raise ValueError('observed_at must include a timezone')
    return stamp.timestamp()


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def parse_rk_log_line(line, channel):
    """Parse only verified channels; never infer a VOC mapping or physical unit."""
    channels = {'CO': (10, 0), 'O2': (5, 0), 'temperature': (3, 0), 'humidity': (3, 1)}
    if channel not in channels:
        raise ValueError('RK log adapter has no verified mapping for '+str(channel))
    if 'PtlGasSensor_RK_GAS01.cpp' not in line:
        return None
    match = re.search(r'RECV\s*\[([0-9A-Fa-f]+)\]', line)
    if not match:
        return None
    packet = bytes.fromhex(match[1])
    if len(packet) != 12 or packet[:4] != bytes.fromhex('5B5B0C01'):
        raise ValueError('Unsupported RK reply layout')
    number, error, power = packet[4:7]
    if number != channels[channel][0]:
        return None
    if (binascii.crc_hqx(packet[:-1], 0) & 255) != packet[-1]:
        raise ValueError('Sensor reply checksum failed')
    if error:
        raise ValueError('Sensor reply error code '+str(error))
    if power not in (0, 1, 2, 3, 4):
        raise ValueError('Unsupported RK scaling code')
    if number == 3:
        values = [int.from_bytes(packet[7:9], 'little', signed=True),
                  int.from_bytes(packet[9:11], 'little')]
    else:
        values = [int.from_bytes(packet[7:11], 'little', signed=True)]
    raw = values[channels[channel][1]]
    value = raw
    if power == 1:
        value = raw*10
    elif power in (3, 4):
        value = (abs(raw)//(10 if power == 3 else 100))*(-1 if raw < 0 else 1)
    # Local adapter runs on the SAME host that writes this timestamp.
    stamp = datetime.strptime(line[:19], '%Y-%m-%d %H:%M:%S').astimezone().isoformat()
    return dict(channel=channel, value=value, unit='software_value', valid=True,
                observed_at=stamp, sample_id=hashlib.sha256((channel+stamp+packet.hex()).encode()).hexdigest(),
                source='local_rk_log', timestamp_basis='local_host_log_time_not_sensor_exposure',
                measurement_verified=False, raw_value=raw, gas_power=power,
                packet_hex=packet.hex().upper(), source_log_time=line[:19], raw_log=line.rstrip())


class RKGasLogReader:
    """Wait for a NEW matching log line after this read starts, on host .120.

    No SSH/password prompt, background polling, or competing sensor connection.
    The robot must remain stopped while waiting (observed cadence ~60 seconds).
    """
    def __init__(self, channel, log_path='/home/root/log/other.log', timeout_s=75, poll_s=.1):
        if channel not in ('CO', 'O2', 'temperature', 'humidity'):
            raise ValueError('VOC log mapping is not verified; supply a VOC reader/sample instead')
        if not all(math.isfinite(v) and v > 0 for v in (timeout_s, poll_s)):
            raise ValueError('Log reader timeouts must be finite and positive')
        self.channel, self.log_path = channel, Path(log_path)
        self.timeout_s, self.poll_s = timeout_s, poll_s

    def __call__(self):
        started = time.monotonic()
        stream = self.log_path.open('rb')
        try:
            stream.seek(0, os.SEEK_END)  # Old lines cannot be attributed to this stop.
            identity = (os.fstat(stream.fileno()).st_dev, os.fstat(stream.fileno()).st_ino)
            pending = b''
            while time.monotonic()-started < self.timeout_s:
                try:
                    stat = self.log_path.stat()
                except FileNotFoundError:
                    time.sleep(min(self.poll_s, max(0, self.timeout_s-(time.monotonic()-started))))
                    continue
                if (stat.st_dev, stat.st_ino) != identity:
                    stream.close()
                    stream = self.log_path.open('rb')
                    identity = (os.fstat(stream.fileno()).st_dev, os.fstat(stream.fileno()).st_ino)
                    pending = b''
                elif stat.st_size < stream.tell():
                    stream.seek(0)
                    pending = b''
                chunk = stream.readline(65536)
                if chunk:
                    pending += chunk
                    if len(pending) > 65536:
                        raise ValueError('Oversized sensor log line')
                    if pending.endswith(b'\n'):
                        result = parse_rk_log_line(pending.decode('utf-8', errors='replace'), self.channel)
                        pending = b''
                        if result is not None:
                            return result
                else:
                    time.sleep(min(self.poll_s, max(0, self.timeout_s-(time.monotonic()-started))))
            raise TimeoutError('No new '+self.channel+' reading before log timeout')
        finally:
            stream.close()


class GasAlarmDetector:
    """A configurable sensor rule, not an environmental safety assessment.

    reader: optional callable returning the documented sample dictionary. Device
    adapters must bound their own I/O time. An explicit sample can also be passed.
    All samples require channel, unit, value, valid, sample_id and observed_at.
    """
    def __init__(self, channel, threshold, unit, reader=None, comparison='high',
                 output_dir='p4_results', max_age_s=90, future_tolerance_s=5,
                 repeat_warning_s=60, speak=False, speech_timeout_s=30,
                 audio_device='plughw:CARD=MCP01,DEV=0', rule_purpose='user_configured'):
        if not isinstance(channel, str) or not channel.strip() or len(channel) > 64:
            raise ValueError('A sensor channel is required')
        if not isinstance(unit, str) or not unit.strip() or len(unit) > 32:
            raise ValueError('An explicit unit is required')
        if isinstance(threshold, bool) or not math.isfinite(float(threshold)):
            raise ValueError('A finite explicit threshold is required')
        if comparison not in ('high', 'low'):
            raise ValueError('comparison must be high (>=) or low (<=)')
        if rule_purpose not in ('integration_test_only', 'user_configured'):
            raise ValueError('Unknown rule_purpose')
        for value in (max_age_s, speech_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Timeouts must be finite and positive')
        for value in (future_tolerance_s, repeat_warning_s):
            if not math.isfinite(value) or value < 0:
                raise ValueError('Tolerances must be finite and nonnegative')
        if reader is not None and not callable(reader):
            raise ValueError('reader must be callable')
        self.channel, self.unit, self.threshold = channel, unit, float(threshold)
        self.reader, self.comparison = reader, comparison
        self.rule_purpose = rule_purpose
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.max_age_s, self.future_tolerance_s = max_age_s, future_tolerance_s
        self.repeat_warning_s, self.speak = repeat_warning_s, speak
        self.speech_timeout_s, self.audio_device = speech_timeout_s, audio_device
        self.session_id = uuid.uuid4().hex
        self._episodes, self._seen, self._last_observed = {}, set(), {}
        self._lock = threading.Lock()

    @classmethod
    def from_config(cls, config_path=None, reader=None, **overrides):
        """Load the adjacent CO preset, or a supplied JSON configuration.

        Initialization never reads a sensor. A custom reader overrides the local
        log adapter; its channel/unit must still agree with the configuration.
        """
        path = Path(config_path) if config_path is not None else Path(__file__).with_name('P4配置.json')
        path = path.expanduser().resolve()
        config = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(config, dict):
            raise ValueError('P4 configuration must be a JSON object')
        config.update(overrides)
        source = config.pop('source', 'rk_log')
        if source not in ('rk_log', 'external'):
            raise ValueError('source must be rk_log or external')
        log_path = Path(config.pop('log_path', '/home/root/log/other.log')).expanduser()
        timeout = config.pop('wait_timeout_s', 75)
        if source == 'rk_log' and reader is None:
            if config.get('unit') != 'software_value':
                raise ValueError('RK log values have unverified units; use software_value or supply a verified adapter')
            if not log_path.is_absolute():
                log_path = path.parent/log_path
            reader = RKGasLogReader(config['channel'], log_path, timeout_s=timeout)
        output = Path(config.get('output_dir', 'p4_results')).expanduser()
        config['output_dir'] = output if output.is_absolute() else path.parent/output
        return cls(reader=reader, **config)

    def reset(self):
        """Begin a new observation session; keep recorded evidence on disk."""
        if not self._lock.acquire(blocking=False):
            return {'status': 'busy'}
        try:
            self._episodes, self._seen, self._last_observed = {}, set(), {}
            self.session_id = uuid.uuid4().hex
            return {'status': 'ok', 'session_id': self.session_id}
        finally:
            self._lock.release()

    def _speech(self, text, output):
        command = [sys.executable, str(Path(__file__).resolve()), '--speak-text', text,
                   '--speech-output', str(output), '--audio-device', self.audio_device]
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.PIPE, start_new_session=True)
        try:
            _, errors = process.communicate(timeout=self.speech_timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.communicate()
            raise TimeoutError('Warning speech timed out')
        if process.returncode:
            raise RuntimeError('Warning speech failed: '+errors.decode(errors='replace')[-1000:])

    def detect(self, task_id, robot_pose=None, checkpoint_id=None, location_label=None,
               frame_id='map', sample=None, replay=False):
        """Evaluate one fresh sample, record position/evidence and optionally speak.

        replay=True requires an explicit sample, never calls hardware or audio,
        and ignores historical age only for labeled offline rule testing.
        """
        if not self._lock.acquire(blocking=False):
            return dict(status='busy', anomaly=None, decision='unknown')
        mode = 'offline_replay' if replay else 'live'
        result = dict(status='error', anomaly=None, decision='unknown', session_id=self.session_id,
                      mode=mode, started_at=_now(), observation_pose=None, location_valid=False,
                      warning_text=None, speech_status='not_requested', warning_emitted=False,
                      alarm_id=None, evidence={}, warnings=[],
                      rule=dict(channel=self.channel, unit=self.unit, threshold=self.threshold,
                                comparison=self.comparison, purpose=self.rule_purpose),
                      location_method='sensor_observation_position_not_leak_source')
        folder, episode, episode_key, seen_key = None, None, None, None
        try:
            checkpoint_id = task_id if checkpoint_id is None else checkpoint_id
            for name, value in (('task_id', task_id), ('checkpoint_id', checkpoint_id), ('frame_id', frame_id)):
                if not isinstance(value, str) or not value.strip() or len(value) > 128:
                    raise ValueError(name+' must contain 1-128 characters')
            if location_label is not None and (not isinstance(location_label, str) or len(location_label) > 100):
                raise ValueError('Invalid location label')
            pose = None
            if robot_pose is not None:
                pose = {key: float(robot_pose[key]) for key in ('x', 'y', 'yaw')}
                if not all(math.isfinite(v) for v in pose.values()):
                    raise ValueError('robot_pose must contain finite x, y, yaw')
            result.update(task_id=task_id, checkpoint_id=checkpoint_id, frame_id=frame_id,
                          location_label=location_label)
            folder = self.output_dir/(datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex)
            folder.mkdir(parents=True, exist_ok=False)
            result['evidence']['result_json'] = str(folder/'result.json')
            if sample is None:
                if replay:
                    raise ValueError('Replay requires an explicit sample')
                if self.reader is None:
                    raise ValueError('No sensor reader/sample configured; VOC interface must be supplied')
                sample = self.reader()
            if not isinstance(sample, dict):
                raise ValueError('Sensor adapter must return a sample dictionary')
            # Preserve malformed input as text too, without accepting NaN as a reading.
            (folder/'raw_sample.txt').write_text(repr(sample)+'\n', encoding='utf-8')
            result['evidence']['raw_sample'] = str(folder/'raw_sample.txt')
            if sample.get('channel') != self.channel:
                raise ValueError('Sensor channel mismatch; cannot substitute another gas for '+self.channel)
            if sample.get('unit') != self.unit:
                raise ValueError('Sensor unit mismatch; no implicit unit conversion')
            if sample.get('valid') is not True:
                raise ValueError('Sensor sample invalid/not ready')
            value = sample.get('value')
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError('Sensor value must be a finite number')
            sample_id = sample.get('sample_id')
            if not isinstance(sample_id, str) or not sample_id or len(sample_id) > 256:
                raise ValueError('A stable sensor sample_id is required')
            observed_epoch = _epoch(sample.get('observed_at'))
            age = time.time()-observed_epoch
            result['sample_age_s'] = age
            if not replay and (age > self.max_age_s or age < -self.future_tolerance_s):
                raise ValueError('Sensor timestamp is stale or in the future; verify sample freshness/clocks')
            seen_key = (mode, sample_id)
            if seen_key in self._seen:
                raise ValueError('Duplicate sample_id: no new reading; do not attach old data to a new position')
            if not replay and observed_epoch < self._last_observed.get(mode, -math.inf):
                raise ValueError('Out-of-order sensor timestamp; an older reading cannot replace the latest observation')
            result.update(sample=dict(channel=self.channel, value=value, unit=self.unit,
                                      sample_id=sample_id, observed_at=sample['observed_at'],
                                      source=sample.get('source', 'caller_adapter'),
                                      timestamp_basis=sample.get('timestamp_basis', 'caller_provided'),
                                      measurement_verified=sample.get('measurement_verified') is True),
                          received_at=_now(), observation_pose=pose, location_valid=pose is not None)
            if self.unit == 'software_value':
                result['warnings'].append('Only a software-value rule; physical units and measurement validity are unverified')
            if self.rule_purpose == 'integration_test_only':
                result['warnings'].append('Temporary integration threshold, not a gas concentration safety limit')
            if replay:
                result['warnings'].append('Offline replay; age check skipped, pose is replay metadata, no actual warning sound')
            anomaly = value >= self.threshold if self.comparison == 'high' else value <= self.threshold
            episode_key = (mode, checkpoint_id, frame_id)
            episode = copy.deepcopy(self._episodes.get(episode_key, dict(active=False, alarm_id=None, last_spoken=None)))
            if anomaly:
                if not episode['active']:
                    episode.update(active=True, alarm_id=uuid.uuid4().hex, last_spoken=None)
                result['alarm_id'] = episode['alarm_id']
                where = location_label or checkpoint_id
                if pose is not None:
                    where += '，观测位置横坐标{:.1f}米，纵坐标{:.1f}米'.format(pose['x'], pose['y'])
                reading_text = '软件读数' if self.unit == 'software_value' else '读数'
                relation = '达到或超过' if self.comparison == 'high' else '达到或低于'
                unit_text = '' if self.unit == 'software_value' else self.unit
                prefix = '联调警告' if self.rule_purpose == 'integration_test_only' else '警告'
                message = prefix+'，{}，{}{}为{:g}{}，{}配置阈值{:g}{}，已记录。'.format(
                    where, self.channel, reading_text, value, unit_text, relation, self.threshold, unit_text)
                result.update(warning_text=message, decision='threshold_triggered')
                if replay:
                    result['speech_status'] = 'disabled_for_replay'
                elif self.speak:
                    if episode['last_spoken'] is not None and time.monotonic()-episode['last_spoken'] < self.repeat_warning_s:
                        result['speech_status'] = 'suppressed_repeated'
                    else:
                        try:
                            self._speech(message, folder/'warning.wav')
                            result.update(speech_status='success', warning_emitted=True)
                            result['evidence']['warning_wav'] = str(folder/'warning.wav')
                            episode['last_spoken'] = time.monotonic()
                        except Exception as exc:
                            result['speech_status'] = 'failed'
                            result['warnings'].append(str(exc))
            else:
                episode.update(active=False, alarm_id=None, last_spoken=None)
                result.update(decision='not_triggered', speech_status='not_needed')
            result.update(status='ok', anomaly=anomaly)
            _write_json(folder/'reading.json', result['sample'])
            result['evidence']['reading_json'] = str(folder/'reading.json')
            with (folder/'reading.csv').open('w', newline='', encoding='utf-8-sig') as stream:
                row = dict(observed_at=sample['observed_at'], channel=self.channel, value=value, unit=self.unit,
                           threshold=self.threshold, comparison=self.comparison, anomaly=anomaly,
                           checkpoint_id=checkpoint_id, frame_id=frame_id, x=pose['x'] if pose else '',
                           y=pose['y'] if pose else '', yaw=pose['yaw'] if pose else '',
                           sample_id=sample_id, alarm_id=result['alarm_id'])
                writer = csv.DictWriter(stream, fieldnames=list(row))
                writer.writeheader()
                writer.writerow(row)
            result['evidence']['reading_csv'] = str(folder/'reading.csv')
        except Exception as exc:
            result.update(status='error', anomaly=None, decision='unknown',
                          error=str(exc), error_type=type(exc).__name__)
        finally:
            result['finished_at'] = _now()
            try:
                if folder is not None:
                    _write_json(folder/'result.json', result)
                    if result['status'] == 'ok':
                        self._episodes[episode_key] = episode
                        self._seen.add(seen_key)
                        self._last_observed[mode] = observed_epoch
            except Exception as exc:
                result.update(status='error', anomaly=None, decision='unknown', error='Result persistence failed: '+str(exc))
            finally:
                self._lock.release()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='Defaults to adjacent P4配置.json (CO integration preset)')
    parser.add_argument('--channel')
    parser.add_argument('--threshold', type=float)
    parser.add_argument('--unit')
    parser.add_argument('--comparison', choices=('high', 'low'))
    parser.add_argument('--sample-json', type=Path, help='Explicit normalized sensor reading')
    parser.add_argument('--replay', action='store_true', help='Offline test; never reads hardware or plays audio')
    parser.add_argument('--log-path', type=Path, help='Local .120 RK log; wait for NEW matching reply')
    parser.add_argument('--wait-timeout', type=float)
    parser.add_argument('--task-id', default='p4_observation')
    parser.add_argument('--checkpoint-id')
    parser.add_argument('--location-label')
    parser.add_argument('--pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW_RAD'))
    parser.add_argument('--frame-id', default='map')
    parser.add_argument('--output-dir', type=Path)
    speech = parser.add_mutually_exclusive_group()
    speech.add_argument('--speak', action='store_true', default=None)
    speech.add_argument('--no-speak', action='store_false', dest='speak')
    parser.add_argument('--audio-device')
    parser.add_argument('--speak-text', help=argparse.SUPPRESS)
    parser.add_argument('--speech-output', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.speak_text is not None:
        if args.speech_output is None or not args.speak_text.strip() or len(args.speak_text) > 500:
            parser.error('Invalid speech arguments')
        info = synthesize(args.speak_text, args.speech_output)
        subprocess.run(['aplay', '-D', args.audio_device or 'plughw:CARD=MCP01,DEV=0', str(args.speech_output)],
                       stdin=subprocess.DEVNULL, check=True, timeout=info['duration_seconds']+10)
        return 0
    if args.sample_json and args.log_path:
        parser.error('Choose only one of --sample-json or --log-path')
    if args.replay and not args.sample_json:
        parser.error('--replay requires --sample-json')
    overrides = {key: value for key, value in dict(channel=args.channel, threshold=args.threshold,
                 unit=args.unit, comparison=args.comparison, output_dir=args.output_dir,
                 speak=args.speak, audio_device=args.audio_device, log_path=args.log_path,
                 wait_timeout_s=args.wait_timeout).items() if value is not None}
    if args.sample_json:
        overrides['source'] = 'external'
    elif args.log_path:
        overrides['source'] = 'rk_log'
    detector = GasAlarmDetector.from_config(args.config, **overrides)
    sample = json.loads(args.sample_json.read_text(encoding='utf-8')) if args.sample_json else None
    pose = dict(zip(('x', 'y', 'yaw'), args.pose)) if args.pose else None
    result = detector.detect(args.task_id, pose, args.checkpoint_id, args.location_label,
                             args.frame_id, sample, args.replay)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result['status'] == 'ok' else 1


# Reused from the verified local speech helper.
def synthesize(text, output):
    library = ctypes.util.find_library('espeak-ng')
    if not library:
        raise RuntimeError('libespeak-ng is not installed')
    lib = c.CDLL(library)
    callback_type = c.CFUNCTYPE(c.c_int, c.POINTER(c.c_short), c.c_int, c.c_void_p)
    lib.espeak_Initialize.argtypes = [c.c_int, c.c_int, c.c_char_p, c.c_int]
    lib.espeak_Initialize.restype = c.c_int
    lib.espeak_SetVoiceByName.argtypes = [c.c_char_p]
    lib.espeak_SetVoiceByName.restype = c.c_int
    lib.espeak_SetSynthCallback.argtypes = [callback_type]
    lib.espeak_SetSynthCallback.restype = None
    lib.espeak_Synth.argtypes = [c.c_void_p, c.c_size_t, c.c_uint, c.c_int,
                                c.c_uint, c.c_uint, c.POINTER(c.c_uint), c.c_void_p]
    lib.espeak_Synth.restype = c.c_int
    lib.espeak_Terminate.argtypes = []
    lib.espeak_Terminate.restype = c.c_int
    rate = lib.espeak_Initialize(2, 0, None, 0x8000)
    if rate <= 0:
        raise RuntimeError('Speech initialization failed')
    chunks = []
    @callback_type
    def callback(samples, count, events):
        if samples and count > 0:
            chunks.append(c.string_at(samples, count*2))
        return 0
    try:
        # Older installed versions name Mandarin "zh"; newer versions also use "cmn".
        for voice in (b'cmn', b'zh'):
            if lib.espeak_SetVoiceByName(voice) == 0:
                break
        else:
            raise RuntimeError('Mandarin voice unavailable')
        lib.espeak_SetSynthCallback(callback)
        data = c.create_string_buffer(text.encode('utf-8'))
        status = lib.espeak_Synth(data, len(data), 0, 1, 0, 1, None, None)
        if status != 0:
            raise RuntimeError('Speech synthesis failed: '+str(status))
    finally:
        lib.espeak_Terminate()
    pcm = b''.join(chunks)
    if not pcm or not any(pcm):
        raise RuntimeError('Empty or silent synthesis')
    with wave.open(str(output), 'wb') as wav:
        wav.setparams((1, 2, rate, 0, 'NONE', 'not compressed'))
        wav.writeframes(pcm)
    return dict(text=text, wav=str(output), voice=voice.decode(), sample_rate=rate,
                duration_seconds=len(pcm)/2/rate, synthesis='success')

if __name__ == "__main__":
    sys.exit(main())
