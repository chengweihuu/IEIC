"""Offline tests only: no robot connections, motion, or audio."""
import functools
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from contextlib import ExitStack

import numpy as np

from p2_rescue import RescueDetector, read_fir


REAL_FIR = (Path(__file__).resolve().parents[1] / '测试结果' /
            '2026-10-12_云台测温接口/红外矩阵/20230619033310_ir.fir')


def write_fir(path, matrix):
    header = bytearray(64)
    header[:4] = b'FIR\0'
    struct.pack_into('<HHff', header, 24, 512, 640, float(matrix.max()), float(matrix.min()))
    path.write_bytes(header + np.rint(matrix*10).astype('<i2').tobytes())


class RescueTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.fir = self.root/'input.fir'
        self.matrix = np.full((512, 640), 25.0)
        self.matrix[200:220, 300:320] = 35
        self.matrix[20:25, 300:305] = 36  # Reject small hot noise.
        self.matrix[100:150, 20:70] = 38  # Left-side target in the default full frame.
        write_fir(self.fir, self.matrix)

    def tearDown(self):
        self.temp.cleanup()

    def detector(self, **kwargs):
        return RescueDetector(output_dir=self.root/'results', **kwargs)

    def test_real_fir_negative_and_saved_evidence(self):
        matrix = read_fir(REAL_FIR)
        self.assertEqual(matrix.shape, (512, 640))
        self.assertAlmostEqual(float(matrix.max()), 28.2)
        result = self.detector().detect('real', fir_path=REAL_FIR)
        self.assertEqual(result['status'], 'ok')
        self.assertFalse(result['target_found'])
        self.assertFalse(result['location_valid'])
        self.assertEqual(json.loads(Path(result['evidence']['result_json']).read_text()), result)
        image = Path(result['evidence']['thermal_image']).read_bytes()
        self.assertEqual(image[:8], b'\x89PNG\r\n\x1a\n')

    def test_heat_source_roi_noise_coordinates_and_replay_isolation(self):
        detector = self.detector(speak=True, assumed_distance_m=2, horizontal_fov_deg=60, roi=(1/3, 0, 2/3, 1))
        with patch.object(detector, '_capture', side_effect=AssertionError('network forbidden')), \
             patch.object(detector, '_speech', side_effect=AssertionError('audio forbidden')), \
             patch.object(detector, '_visible', side_effect=AssertionError('network forbidden')):
            result = detector.detect('synthetic', {'x': 5, 'y': 3, 'yaw': math.pi/2}, fir_path=self.fir)
        self.assertTrue(result['target_found'])
        self.assertEqual(result['region_count'], 1)
        self.assertEqual(result['regions'][0]['area_pixels'], 400)
        self.assertEqual(result['regions'][0]['bbox_xywh'], [300, 200, 20, 20])
        self.assertEqual(result['regions'][0]['centroid_xy'], [309.5, 209.5])
        self.assertLess(result['estimated_target_position']['x'], 5)
        self.assertAlmostEqual(math.hypot(result['estimated_target_position']['x']-5, result['estimated_target_position']['y']-3), 2)
        self.assertFalse(result['estimated_target_position']['distance_measured'])
        self.assertEqual(result['speech_status'], 'disabled_for_replay')

    def test_no_assumed_coordinate_by_default(self):
        result = self.detector().detect('test', {'x': 1, 'y': 2, 'yaw': 0}, fir_path=self.fir)
        self.assertTrue(result['target_found'])
        self.assertIsNone(result['estimated_target_position'])

    def test_full_frame_includes_second_region(self):
        result = self.detector(roi=(0, 0, 1, 1)).detect('full', fir_path=self.fir)
        self.assertEqual(result['region_count'], 2)

    def test_bad_data_is_error_not_negative_and_lock_releases(self):
        self.fir.write_bytes(b'broken')
        detector = self.detector()
        result = detector.detect('bad', fir_path=self.fir)
        self.assertEqual(result['status'], 'error')
        self.assertIsNone(result['target_found'])
        self.assertIn('FIR', result['error'])
        result = detector.detect('next', fir_path=REAL_FIR)
        self.assertEqual(result['status'], 'ok')

    def test_header_mismatch_and_truncation(self):
        data = bytearray(self.fir.read_bytes())
        struct.pack_into('<f', data, 28, 100)
        self.fir.write_bytes(data)
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            read_fir(self.fir)
        self.fir.write_bytes(data[:-1])
        with self.assertRaisesRegex(ValueError, 'size'):
            read_fir(self.fir)

    def test_invalid_pose_never_contacts_camera(self):
        detector = self.detector()
        with patch.object(detector, '_capture') as capture:
            result = detector.detect('bad', {'x': float('nan'), 'y': 0, 'yaw': 0})
        capture.assert_not_called()
        self.assertEqual(result['status'], 'error')
        self.assertIsNone(result['target_found'])
        json.dumps(result, allow_nan=False)

    def test_busy(self):
        detector = self.detector()
        detector._lock.acquire()
        try:
            result = detector.detect('busy')
            self.assertEqual(result['status'], 'busy')
            self.assertIsNone(result['target_found'])
        finally:
            detector._lock.release()

    def test_live_optional_failures_do_not_erase_positive_detection(self):
        detector = self.detector(speak=True)
        def download(remote, target):
            if remote.endswith('.fir'):
                target.write_bytes(self.fir.read_bytes())
            else:
                raise RuntimeError('jpg unavailable')
        with patch.object(detector, '_capture', return_value={'ir_fir_path': '/test.fir', 'ir_pic_path': '/test.jpg'}), \
             patch.object(detector, '_download', side_effect=download), \
             patch.object(detector, '_visible', side_effect=RuntimeError('ffmpeg failed')), \
             patch.object(detector, '_speech', side_effect=RuntimeError('audio failed')):
            result = detector.detect('live_mock')
        self.assertEqual(result['status'], 'ok')
        self.assertTrue(result['target_found'])
        self.assertEqual(result['speech_status'], 'failed')
        self.assertEqual(len(result['warnings']), 4)

    def test_download_failure_is_error(self):
        detector = self.detector()
        with patch.object(detector, '_capture', return_value={'ir_fir_path': '/test.fir'}), \
             patch.object(detector, '_download', side_effect=TimeoutError('SSH timeout')):
            result = detector.detect('live_mock')
        self.assertEqual(result['status'], 'error')
        self.assertIsNone(result['target_found'])

    def test_remote_path_rejection(self):
        with patch('p2_rescue.subprocess.run') as run:
            with self.assertRaises(ValueError):
                self.detector()._download('/mnt/sdcard/data/notask/a.fir;id', self.root/'out')
        run.assert_not_called()

    def test_protocol_capture_and_corruption(self):
        class FakeSocket:
            def __init__(self, reply):
                self.reply = reply
                self.sent = None
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
            def settimeout(self, timeout):
                pass
            def sendall(self, frame):
                self.sent = frame
            def recv(self, size):
                # Fragmented reads exercise exact-length handling.
                count = min(size, 3)
                part, self.reply = self.reply[:count], self.reply[count:]
                return part
        for corrupt in (False, True):
            files = {'ir_fir_path': '/mnt/sdcard/data/notask/a.fir'}
            body = bytes.fromhex('0500000000') + struct.pack('<I', 2) + json.dumps(files).encode()
            checksum = functools.reduce(int.__xor__, body, 0) ^ int(corrupt)
            reply = bytes.fromhex('f0e0')+struct.pack('<HH', 51000, len(body))+body+bytes([checksum])+bytes.fromhex('f1e1')
            sock = FakeSocket(reply)
            with patch('p2_rescue.socket.create_connection', return_value=sock):
                if corrupt:
                    with self.assertRaisesRegex(ValueError, 'checksum'):
                        self.detector()._capture()
                else:
                    self.assertEqual(self.detector()._capture(), files)
            self.assertEqual(sock.sent[6:9], bytes.fromhex('050000'))
            self.assertEqual(struct.unpack('<ii', sock.sent[9:17]), (2, 0))

    def test_camera_timeout(self):
        with patch('p2_rescue.socket.create_connection') as connect:
            connect.return_value.__enter__.return_value.recv.side_effect = TimeoutError('timed out')
            with self.assertRaises(TimeoutError):
                self.detector(camera_timeout_s=.02)._capture()

    def test_speech_timeout_cleans_playback_process_group(self):
        with patch('p2_rescue.subprocess.Popen') as popen, patch('p2_rescue.os.killpg') as kill:
            process = popen.return_value
            process.pid = 12345
            process.communicate.side_effect = [subprocess.TimeoutExpired('speech', 30), (None, b'')]
            with self.assertRaisesRegex(TimeoutError, 'Speech'):
                self.detector()._speech('测试', self.root/'speech.wav')
            kill.assert_called_once()
            self.assertEqual(kill.call_args.args[0], 12345)


class RepeatedObservationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.fir = self.root/'scene.fir'
        self.pose = {'x': 5, 'y': 3, 'yaw': 0}

    def tearDown(self):
        self.temp.cleanup()

    def scene(self, boxes):
        matrix = np.full((512, 640), 25.0)
        for x, y, w, h in boxes:
            matrix[y:y+h, x:x+w] = 35
        write_fir(self.fir, matrix)

    def detector(self, **kwargs):
        return RescueDetector(output_dir=self.root/'out', capture_visible=False, **kwargs)

    def replay(self, detector, **kwargs):
        options = dict(task_id='observation', checkpoint_id='A', robot_pose=self.pose, fir_path=self.fir)
        options.update(kwargs)
        result = detector.detect(**options)
        self.assertEqual(result['status'], 'ok', result.get('error'))
        return result

    def live_mocks(self, detector, stack):
        stack.enter_context(patch.object(detector, '_capture', return_value={
            'ir_fir_path': '/scene.fir', 'ir_pic_path': '/scene.jpg'}))
        def download(remote, path):
            path.write_bytes(self.fir.read_bytes() if remote.endswith('.fir') else b'fixture-jpg')
        stack.enter_context(patch.object(detector, '_download', side_effect=download))
        return stack.enter_context(patch.object(detector, '_speech'))

    def test_full_frame_three_directions_and_numbering(self):
        self.scene([(20, 100, 20, 20), (300, 100, 20, 20), (590, 100, 20, 20)])
        result = self.replay(self.detector())
        self.assertEqual(result['candidate_count'], 3)
        self.assertEqual([t['direction'] for t in result['targets']], ['left_front', 'front', 'right_front'])
        self.assertEqual([t['target_number'] for t in result['targets']], [1, 2, 3])
        self.assertEqual(result['direction'], 'multiple')
        self.assertIsNone(result['unique_person_count'])
        self.assertIsNone(result['new_person_count'])

    def test_near_fragments_merge_before_area_filter(self):
        # Neither fragment alone reaches the 64-pixel candidate threshold.
        self.scene([(100, 100, 6, 6), (100, 114, 6, 6), (500, 100, 10, 10), (20, 20, 2, 2)])
        result = self.replay(self.detector())
        self.assertEqual(result['candidate_count'], 2)
        self.assertEqual(result['targets'][0]['component_count'], 2)
        self.assertEqual(result['targets'][0]['area_pixels'], 72)
        self.assertEqual(result['targets'][0]['bbox_xywh'], [100, 100, 6, 20])

    def test_excessive_noise_returns_error_instead_of_spurious_negative(self):
        self.scene([(x, y, 3, 3) for y in range(0, 260, 10) for x in range(0, 260, 10)])
        result = self.detector().detect('noise', robot_pose=self.pose, fir_path=self.fir)
        self.assertEqual(result['status'], 'error')
        self.assertIsNone(result['target_found'])
        self.assertIn('Too many hot fragments', result['error'])

    def test_shift_same_point_keeps_id_even_with_new_task_id(self):
        self.scene([(100, 100, 20, 20)])
        detector = self.detector()
        first = self.replay(detector)
        self.scene([(107, 104, 20, 20)])
        second = self.replay(detector, task_id='different_call_id')
        self.assertEqual(first['targets'][0]['target_id'], second['targets'][0]['target_id'])
        self.assertEqual(second['targets'][0]['association'], 'matched_in_view')
        self.assertEqual(second['new_candidate_count_in_view'], 0)

    def test_new_local_candidate_and_return_after_missing_frame(self):
        detector = self.detector()
        self.scene([(100, 100, 20, 20)])
        first = self.replay(detector)
        self.scene([])
        self.assertFalse(self.replay(detector)['target_found'])
        self.scene([(100, 100, 20, 20), (500, 100, 20, 20)])
        last = self.replay(detector)
        self.assertEqual(last['targets'][0]['target_id'], first['targets'][0]['target_id'])
        self.assertEqual(last['new_candidate_count_in_view'], 1)

    def test_turn_move_checkpoint_frame_and_missing_pose_do_not_increment_people(self):
        self.scene([(100, 100, 20, 20)])
        for change in ({'robot_pose': {'x': 5, 'y': 3, 'yaw': .5}},
                       {'robot_pose': {'x': 6, 'y': 3, 'yaw': 0}},
                       {'checkpoint_id': 'B'}, {'frame_id': 'other_map'}, {'robot_pose': None}):
            with self.subTest(change=change):
                detector = self.detector()
                first = self.replay(detector)
                second = self.replay(detector, **change)
                self.assertNotEqual(first['view_id'], second['view_id'])
                self.assertEqual(second['targets'][0]['association'], 'unconfirmed_observation')
                self.assertTrue(second['targets'][0]['needs_confirmation'])
                self.assertEqual(second['new_candidate_count_in_view'], 0)
                self.assertIsNone(second['new_person_count'])

    def test_fixed_anchor_prevents_cumulative_drift(self):
        self.scene([(100, 100, 20, 20)])
        detector = self.detector()
        first = self.replay(detector)
        second = self.replay(detector, robot_pose={'x': 5.2, 'y': 3, 'yaw': 0})
        third = self.replay(detector, robot_pose={'x': 5.4, 'y': 3, 'yaw': 0})
        self.assertEqual(first['view_id'], second['view_id'])
        self.assertNotEqual(first['view_id'], third['view_id'])

    def test_yaw_wraparound_matches_and_return_to_previous_view(self):
        self.scene([(100, 100, 20, 20)])
        detector = self.detector()
        first = self.replay(detector, robot_pose={'x': 5, 'y': 3, 'yaw': math.pi-.02})
        wrapped = self.replay(detector, robot_pose={'x': 5, 'y': 3, 'yaw': -math.pi+.02})
        self.assertEqual(first['targets'][0]['target_id'], wrapped['targets'][0]['target_id'])
        self.replay(detector, robot_pose={'x': 5, 'y': 3, 'yaw': 0})
        returned = self.replay(detector, robot_pose={'x': 5, 'y': 3, 'yaw': math.pi-.02})
        self.assertEqual(first['targets'][0]['target_id'], returned['targets'][0]['target_id'])

    def test_split_does_not_create_two_new_ids(self):
        self.scene([(100, 100, 100, 20)])
        detector = self.detector(merge_gap_pixels=5)
        first = self.replay(detector)
        self.scene([(100, 100, 25, 20), (175, 100, 25, 20)])
        split = self.replay(detector)
        self.assertEqual(split['candidate_count'], 2)
        self.assertEqual(split['new_candidate_count_in_view'], 0)
        for target in split['targets']:
            self.assertIsNone(target['target_id'])
            self.assertEqual(target['association'], 'ambiguous_local_match')
            self.assertEqual(target['possible_target_ids'], [first['targets'][0]['target_id']])

    def test_two_old_targets_merged_are_ambiguous(self):
        self.scene([(100, 100, 25, 20), (175, 100, 25, 20)])
        detector = self.detector(merge_gap_pixels=5)
        first = self.replay(detector)
        self.scene([(100, 100, 100, 20)])
        merged = self.replay(detector)
        self.assertIsNone(merged['targets'][0]['target_id'])
        self.assertEqual(len(merged['targets'][0]['possible_target_ids']), 2)
        self.assertEqual(merged['new_candidate_count_in_view'], 0)

    def test_expiry_marks_new_observation_unconfirmed(self):
        self.scene([(100, 100, 20, 20)])
        detector = self.detector(match_max_age_s=10)
        with patch('p2_rescue.time.monotonic', return_value=100):
            first = self.replay(detector)
        with patch('p2_rescue.time.monotonic', return_value=111):
            second = self.replay(detector)
        self.assertNotEqual(first['view_id'], second['view_id'])
        self.assertTrue(second['targets'][0]['needs_confirmation'])
        self.assertEqual(second['new_candidate_count_in_view'], 0)

    def test_repeat_suppression_new_target_and_failed_speech_retry(self):
        self.scene([(100, 100, 20, 20)])
        detector = self.detector(speak=True)
        with ExitStack() as stack:
            speak = self.live_mocks(detector, stack)
            first = self.replay(detector, fir_path=None)
            self.assertTrue(first['announcement_complete'])
            second = self.replay(detector, fir_path=None)
            self.assertEqual(second['speech_status'], 'suppressed_repeated')
            self.assertEqual(speak.call_count, 1)
            self.scene([(100, 100, 20, 20), (500, 100, 20, 20)])
            speak.side_effect = RuntimeError('speaker unavailable')
            third = self.replay(detector, fir_path=None)
            self.assertEqual(third['speech_status'], 'partial')
            self.assertFalse(third['announcement_complete'])
            speak.side_effect = None
            fourth = self.replay(detector, fir_path=None)
            self.assertTrue(fourth['announcement_complete'])
            self.assertEqual(speak.call_count, 3)
            self.assertEqual(fourth['targets'][0]['speech_status'], 'already_announced')
            self.assertEqual(fourth['targets'][1]['speech_status'], 'success')

    def test_announcement_limit_defers_not_drops_targets(self):
        self.scene([(100, 100, 20, 20), (500, 100, 20, 20)])
        detector = self.detector(speak=True, max_announcements=1)
        with ExitStack() as stack:
            speak = self.live_mocks(detector, stack)
            first = self.replay(detector, fir_path=None)
            self.assertEqual(first['targets'][1]['speech_status'], 'deferred')
            second = self.replay(detector, fir_path=None)
            self.assertTrue(second['announcement_complete'])
            self.assertEqual(speak.call_count, 2)

    def test_replay_and_live_have_separate_history(self):
        self.scene([(100, 100, 20, 20)])
        detector = self.detector(speak=True)
        replay = self.replay(detector)
        with ExitStack() as stack:
            speak = self.live_mocks(detector, stack)
            live = self.replay(detector, fir_path=None)
            self.assertEqual(speak.call_count, 1)
            self.assertEqual(live['targets'][0]['association'], 'new_candidate_in_view')
            self.assertFalse(live['needs_confirmation'])

    def test_reset_and_json_failure_do_not_leave_suppression_state(self):
        self.scene([(100, 100, 20, 20)])
        detector = self.detector(speak=True)
        with ExitStack() as stack:
            speak = self.live_mocks(detector, stack)
            with patch('p2_rescue._json', side_effect=OSError('disk full')):
                failed = detector.detect('call', self.pose, checkpoint_id='A', fir_path=self.fir)
            self.assertEqual(failed['status'], 'error')
            self.assertEqual(detector._states, {})
            self.replay(detector, fir_path=None)
            detector.reset(checkpoint_id='A')
            self.replay(detector, fir_path=None)
            self.assertEqual(speak.call_count, 2)
            old_session = detector.session_id
            detector.reset()
            self.assertNotEqual(old_session, detector.session_id)

    def test_flip_and_per_target_estimates(self):
        self.scene([(20, 100, 20, 20), (590, 100, 20, 20)])
        result = self.replay(self.detector(assumed_distance_m=2, horizontal_fov_deg=60))
        self.assertGreater(result['targets'][0]['estimated_target_position']['y'], 3)
        self.assertLess(result['targets'][1]['estimated_target_position']['y'], 3)
        self.assertIsNone(result['estimated_target_position'])
        self.scene([(20, 100, 20, 20)])
        flipped = self.replay(self.detector(flip_horizontal=True))
        self.assertEqual(flipped['targets'][0]['direction'], 'right_front')


if __name__ == '__main__':
    unittest.main()
