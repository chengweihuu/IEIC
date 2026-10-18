"""Offline rule/adapter tests. All numeric thresholds are synthetic test values."""
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from p4_gas_alarm import GasAlarmDetector, RKGasLogReader, parse_rk_log_line
from p4_gas_alarm import main


PACKETS = {
    'CO': '5B5B0C010A00000000000019',
    'O2': '5B5B0C01050001D100000016',
    'temperature': '5B5B0C010300013A01F401D2',
}


def log_line(packet, stamp=None):
    return (stamp or datetime.now().strftime('%Y-%m-%d %H:%M:%S'))+' PtlGasSensor_RK_GAS01.cpp RECV ['+packet+']\n'


class AlarmTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.pose = {'x': 5, 'y': 3, 'yaw': .8}
        self.serial = 0

    def tearDown(self):
        self.temp.cleanup()

    def detector(self, **kwargs):
        options = dict(channel='VOC', unit='test_unit', threshold=10, output_dir=self.root/'results')
        options.update(kwargs)
        return GasAlarmDetector(**options)

    def reading(self, value=12, **kwargs):
        self.serial += 1
        reading = dict(channel='VOC', unit='test_unit', value=value, valid=True,
                       observed_at=datetime.now(timezone.utc).isoformat(),
                       sample_id='synthetic_'+str(self.serial), source='synthetic_fixture')
        reading.update(kwargs)
        return reading

    def detect(self, detector, sample=None, **kwargs):
        options = dict(task_id='test', checkpoint_id='A', robot_pose=self.pose,
                       sample=sample if sample is not None else self.reading())
        options.update(kwargs)
        return detector.detect(**options)

    def test_alarm_records_position_reading_and_warning(self):
        result = self.detect(self.detector())
        self.assertEqual(result['status'], 'ok')
        self.assertTrue(result['anomaly'])
        self.assertEqual(result['observation_pose'], self.pose)
        self.assertIn('警告', result['warning_text'])
        self.assertEqual(json.loads(Path(result['evidence']['result_json']).read_text()), result)
        self.assertTrue(Path(result['evidence']['reading_csv']).exists())
        self.assertFalse(result['warning_emitted'])

    def test_high_low_and_inclusive_threshold(self):
        for comparison, value, anomaly in [('high', 10, True), ('high', 9, False),
                                           ('low', 10, True), ('low', 11, False)]:
            with self.subTest(comparison=comparison, value=value):
                result = self.detect(self.detector(comparison=comparison), self.reading(value))
                self.assertEqual(result['anomaly'], anomaly)

    def test_invalid_and_mismatched_values_are_unknown(self):
        for changed in ({'channel': 'CO'}, {'unit': 'ppm'}, {'valid': False},
                        {'valid': None}, {'value': float('nan')}, {'value': float('inf')},
                        {'value': True}, {'value': '12'}, {'sample_id': ''},
                        {'observed_at': '2026-10-12 10:00:00'}):
            with self.subTest(changed=changed):
                result = self.detect(self.detector(), self.reading(**changed))
                self.assertEqual(result['status'], 'error')
                self.assertIsNone(result['anomaly'])
                self.assertIsNone(result['observation_pose'])

    def test_stale_future_and_duplicate_samples_do_not_record_new_position(self):
        detector = self.detector()
        for offset in (-1000, 1000):
            stamp = (datetime.now(timezone.utc)+timedelta(seconds=offset)).isoformat()
            result = self.detect(detector, self.reading(observed_at=stamp))
            self.assertEqual(result['decision'], 'unknown')
        reading = self.reading()
        self.assertTrue(self.detect(detector, reading)['anomaly'])
        second = self.detect(detector, reading, checkpoint_id='B', robot_pose={'x': 50, 'y': 30, 'yaw': 0})
        self.assertEqual(second['status'], 'error')
        self.assertIn('Duplicate', second['error'])
        self.assertFalse(second['location_valid'])
        self.assertIsNone(second['observation_pose'])

    def test_repeat_warning_recovery_and_new_alarm(self):
        detector = self.detector(speak=True, repeat_warning_s=60)
        with patch.object(detector, '_speech') as speech, patch('p4_gas_alarm.time.monotonic', return_value=100):
            first = self.detect(detector)
            second = self.detect(detector)
            self.assertEqual(second['speech_status'], 'suppressed_repeated')
            self.assertEqual(first['alarm_id'], second['alarm_id'])
            self.assertEqual(speech.call_count, 1)
            normal = self.detect(detector, self.reading(5))
            self.assertFalse(normal['anomaly'])
            third = self.detect(detector)
            self.assertNotEqual(first['alarm_id'], third['alarm_id'])
            self.assertEqual(speech.call_count, 2)

    def test_repeat_interval_elapsed_and_other_checkpoint(self):
        detector = self.detector(speak=True)
        with patch.object(detector, '_speech') as speech:
            with patch('p4_gas_alarm.time.monotonic', return_value=100):
                first = self.detect(detector)
            with patch('p4_gas_alarm.time.monotonic', return_value=161):
                second = self.detect(detector)
                other = self.detect(detector, checkpoint_id='B')
        self.assertEqual(speech.call_count, 3)
        self.assertEqual(first['alarm_id'], second['alarm_id'])
        self.assertNotEqual(second['alarm_id'], other['alarm_id'])

    def test_failed_speech_is_retried_without_losing_anomaly(self):
        detector = self.detector(speak=True)
        with patch.object(detector, '_speech', side_effect=[RuntimeError('audio unavailable'), None]) as speech:
            first = self.detect(detector)
            second = self.detect(detector)
        self.assertTrue(first['anomaly'])
        self.assertEqual(first['speech_status'], 'failed')
        self.assertEqual(first['alarm_id'], second['alarm_id'])
        self.assertTrue(second['warning_emitted'])
        self.assertEqual(speech.call_count, 2)

    def test_bad_reading_does_not_clear_existing_alarm(self):
        detector = self.detector(speak=True)
        with patch.object(detector, '_speech') as speech:
            first = self.detect(detector)
            self.detect(detector, self.reading(0, valid=False))
            last = self.detect(detector)
        self.assertEqual(first['alarm_id'], last['alarm_id'])
        self.assertEqual(speech.call_count, 1)

    def test_delayed_older_normal_sample_cannot_clear_newer_alarm(self):
        detector = self.detector()
        first = self.detect(detector)
        stamp = (datetime.now(timezone.utc)-timedelta(seconds=10)).isoformat()
        old = self.detect(detector, self.reading(0, observed_at=stamp))
        self.assertEqual(old['status'], 'error')
        self.assertIn('Out-of-order', old['error'])
        last = self.detect(detector)
        self.assertEqual(first['alarm_id'], last['alarm_id'])

    def test_replay_never_reads_or_speaks_and_has_separate_history(self):
        reader = lambda: self.fail('Must not read in replay')
        detector = self.detector(reader=reader, speak=True)
        reading = self.reading(observed_at='2023-01-01T00:00:00+00:00')
        with patch.object(detector, '_speech') as speech:
            replay = self.detect(detector, reading, replay=True)
            self.assertEqual(replay['speech_status'], 'disabled_for_replay')
            speech.assert_not_called()
            reading['observed_at'] = datetime.now(timezone.utc).isoformat()
            live = self.detect(detector, reading)
            self.assertEqual(live['speech_status'], 'success')

    def test_missing_reader_and_reader_timeout(self):
        result = self.detector().detect('no_reader')
        self.assertIsNone(result['anomaly'])
        def timeout():
            raise TimeoutError('sensor timeout')
        result = self.detector(reader=timeout).detect('timeout')
        self.assertEqual(result['error_type'], 'TimeoutError')

    def test_busy_reset_and_persistence_failure(self):
        detector = self.detector()
        detector._lock.acquire()
        try:
            self.assertEqual(self.detect(detector)['status'], 'busy')
            self.assertEqual(detector.reset()['status'], 'busy')
        finally:
            detector._lock.release()
        with patch('p4_gas_alarm._write_json', side_effect=OSError('disk full')):
            result = self.detect(detector)
        self.assertEqual(result['status'], 'error')
        self.assertFalse(detector._seen)
        self.assertFalse(detector._episodes)
        old = detector.session_id
        detector.reset()
        self.assertNotEqual(old, detector.session_id)

    def test_software_values_are_named_and_not_calibrated(self):
        detector = self.detector(channel='CO', unit='software_value', threshold=-1)
        sample = parse_rk_log_line(log_line(PACKETS['CO']), 'CO')
        result = self.detect(detector, sample)
        self.assertIn('软件读数', result['warning_text'])
        self.assertFalse(result['sample']['measurement_verified'])
        self.assertTrue(result['warnings'])

    def test_speech_timeout_kills_playback_children(self):
        with patch('p4_gas_alarm.subprocess.Popen') as popen, patch('p4_gas_alarm.os.killpg') as kill:
            process = popen.return_value
            process.pid = 12345
            process.communicate.side_effect = [subprocess.TimeoutExpired('speech', 30), (None, b'')]
            with self.assertRaises(TimeoutError):
                self.detector()._speech('测试', self.root/'speech.wav')
            kill.assert_called_once()


class LogAdapterTests(unittest.TestCase):
    def test_observed_packets(self):
        for channel, packet_key, value in [('CO', 'CO', 0), ('O2', 'O2', 2090),
                                            ('temperature', 'temperature', 3140),
                                            ('humidity', 'temperature', 5000)]:
            with self.subTest(channel=channel):
                parsed = parse_rk_log_line(log_line(PACKETS[packet_key]), channel)
                self.assertEqual(parsed['value'], value)
                self.assertEqual(parsed['unit'], 'software_value')
                self.assertFalse(parsed['measurement_verified'])

    def test_unknown_voc_and_corrupted_packet_rejected(self):
        with self.assertRaisesRegex(ValueError, 'VOC'):
            RKGasLogReader('VOC')
        with self.assertRaises(ValueError):
            parse_rk_log_line(log_line(PACKETS['CO'][:-2]+'00'), 'CO')
        self.assertIsNone(parse_rk_log_line(log_line(PACKETS['CO']), 'O2'))

    def test_tail_ignores_old_lines_and_waits_for_new_complete_line(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'other.log'
            path.write_text(log_line(PACKETS['CO'], '2020-01-01 00:00:00'))
            new_line = log_line(PACKETS['CO'])
            chunks = iter([new_line[:25], new_line[25:]])
            def append(_):
                with path.open('a') as stream:
                    stream.write(next(chunks))
            with patch('p4_gas_alarm.time.sleep', side_effect=append):
                reading = RKGasLogReader('CO', path, timeout_s=1)()
            self.assertNotEqual(reading['source_log_time'], '2020-01-01 00:00:00')

    def test_rotation_and_truncation(self):
        for rotate in (False, True):
            with tempfile.TemporaryDirectory() as temp:
                path = Path(temp)/'other.log'
                path.write_text('old history\n'*100)
                def replace(_):
                    if rotate:
                        path.rename(path.with_suffix('.old'))
                    path.write_text(log_line(PACKETS['CO']))
                with patch('p4_gas_alarm.time.sleep', side_effect=replace):
                    self.assertEqual(RKGasLogReader('CO', path, timeout_s=1)()['value'], 0)

    def test_no_new_data_timeout(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'other.log'
            path.touch()
            with self.assertRaises(TimeoutError):
                RKGasLogReader('CO', path, timeout_s=.01, poll_s=.005)()


class COPresetTests(unittest.TestCase):
    def test_default_preset_connects_co_without_opening_log_at_initialization(self):
        detector = GasAlarmDetector.from_config()
        self.assertEqual(detector.channel, 'CO')
        self.assertEqual(detector.unit, 'software_value')
        self.assertEqual(detector.threshold, 1)
        self.assertEqual(detector.rule_purpose, 'integration_test_only')
        self.assertTrue(detector.speak)
        self.assertIsInstance(detector.reader, RKGasLogReader)
        self.assertEqual(detector.reader.channel, 'CO')

    def test_observed_zero_and_synthetic_positive_under_co_preset(self):
        with tempfile.TemporaryDirectory() as temp:
            detector = GasAlarmDetector.from_config(output_dir=temp)
            zero = parse_rk_log_line(log_line(PACKETS['CO']), 'CO')
            positive = dict(zero, value=2, sample_id='synthetic_co_positive', source='synthetic_fixture')
            with patch.object(detector, '_speech') as speech:
                normal = detector.detect('zero', sample=zero, replay=True)
                alarm = detector.detect('positive', sample=positive, replay=True)
            self.assertFalse(normal['anomaly'])
            self.assertTrue(alarm['anomaly'])
            self.assertIn('CO软件读数', alarm['warning_text'])
            self.assertTrue(alarm['warning_text'].startswith('联调警告'))
            self.assertEqual(alarm['rule']['purpose'], 'integration_test_only')
            speech.assert_not_called()

    def test_config_change_to_another_verified_channel(self):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp)/'config.json'
            config.write_text(json.dumps(dict(source='rk_log', channel='O2', threshold=2000,
                                               unit='software_value', comparison='low',
                                               output_dir='records', log_path='other.log')))
            detector = GasAlarmDetector.from_config(config, speak=False)
            self.assertEqual(detector.channel, 'O2')
            self.assertEqual(detector.comparison, 'low')
            self.assertEqual(detector.output_dir, Path(temp)/'records')
            self.assertEqual(detector.reader.log_path, Path(temp)/'other.log')

    def test_voc_cannot_be_enabled_by_renaming_log_channel(self):
        with self.assertRaisesRegex(ValueError, 'VOC'):
            GasAlarmDetector.from_config(channel='VOC')
        with self.assertRaisesRegex(ValueError, 'software_value'):
            GasAlarmDetector.from_config(unit='ppm')
        reader = lambda: None
        external = GasAlarmDetector.from_config(source='external', channel='VOC', unit='test_unit', reader=reader)
        self.assertIs(external.reader, reader)

    def test_cli_no_speak_overrides_preset_and_can_use_default_rule(self):
        with patch('p4_gas_alarm.GasAlarmDetector.from_config') as factory, \
             patch('sys.argv', ['p4_gas_alarm.py', '--no-speak']), patch('builtins.print'):
            factory.return_value.detect.return_value = {'status': 'ok'}
            self.assertEqual(main(), 0)
            self.assertIs(factory.call_args.kwargs['speak'], False)
        with patch('p4_gas_alarm.GasAlarmDetector.from_config') as factory, \
             patch('sys.argv', ['p4_gas_alarm.py']), patch('builtins.print'):
            factory.return_value.detect.return_value = {'status': 'ok'}
            main()
            self.assertNotIn('speak', factory.call_args.kwargs)


if __name__ == '__main__':
    unittest.main()
