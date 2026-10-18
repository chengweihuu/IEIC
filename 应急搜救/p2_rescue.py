"""P2 thermal search with conservative same-view association. Python >=3.8, NumPy.

No ROS, robot movement, gimbal control, or cross-camera registration.
Run with --help; see P2使用说明.txt for deployment and integration.
"""
import argparse
import copy
import ctypes as c
import ctypes.util
from datetime import datetime
import functools
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
import wave
import zlib

import numpy as np


def _now():
    return datetime.now().astimezone().isoformat(timespec='milliseconds')


def _json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                               allow_nan=False) + '\n', encoding='utf-8')


def read_fir(path):
    """Verified vendor FIR layout: 64-byte header, little-endian int16 / 10 C.

    Preserve file rows. Registration with vendor JPG has NOT been verified.
    """
    raw = Path(path).read_bytes()
    if len(raw) < 64 or raw[:4] != b'FIR\0':
        raise ValueError('Invalid FIR signature/header')
    height, width = struct.unpack_from('<HH', raw, 24)
    if (width, height) != (640, 512) or len(raw) != 64 + width * height * 2:
        raise ValueError('Invalid FIR dimensions/payload size')
    maximum, minimum = struct.unpack_from('<ff', raw, 28)
    matrix = np.frombuffer(raw, dtype='<i2', offset=64).reshape(height, width) / 10.0
    if not all(math.isfinite(v) for v in (maximum, minimum)):
        raise ValueError('Invalid FIR header temperatures')
    if abs(float(matrix.max()) - maximum) > .11 or abs(float(matrix.min()) - minimum) > .11:
        raise ValueError('FIR header/payload temperature mismatch')
    return matrix


def _regions(mask, matrix, min_area):
    """8-connected components; standard-library flood fill avoids OpenCV dependency."""
    remaining = mask.copy()
    height, width = mask.shape
    regions = []
    for sy, sx in zip(*np.nonzero(mask)):
        if not remaining[sy, sx]:
            continue
        remaining[sy, sx] = False
        stack = [(int(sy), int(sx))]
        area = sum_x = sum_y = 0
        x0 = x1 = int(sx)
        y0 = y1 = int(sy)
        maximum = -math.inf
        total = 0.0
        while stack:
            y, x = stack.pop()
            area += 1
            sum_x += x
            sum_y += y
            x0, x1 = min(x0, x), max(x1, x)
            y0, y1 = min(y0, y), max(y1, y)
            temp = float(matrix[y, x])
            maximum, total = max(maximum, temp), total + temp
            for ny in range(max(0, y-1), min(height, y+2)):
                for nx in range(max(0, x-1), min(width, x+2)):
                    if remaining[ny, nx]:
                        remaining[ny, nx] = False
                        stack.append((ny, nx))
        if area >= min_area:
            regions.append(dict(area_pixels=area, bbox_xywh=[x0, y0, x1-x0+1, y1-y0+1],
                                centroid_xy=[sum_x/area, sum_y/area],
                                max_temperature_c=maximum, mean_temperature_c=total/area))
    return sorted(regions, key=lambda v: v['area_pixels'], reverse=True)


def _merge_regions(regions, gap_pixels, min_area):
    """Join nearby component boxes, then filter by actual hot pixel area.

    Proximity is only a heuristic; it is not a person classifier. Transitive
    groups can join adjacent people, so retain component boxes for inspection.
    """
    if len(regions) > 512:
        raise ValueError('Too many hot fragments (>512); check scene/temperature threshold')
    parents = list(range(len(regions)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    for i, first in enumerate(regions):
        x, y, w, h = first['bbox_xywh']
        for j in range(i):
            a, b, c, d = regions[j]['bbox_xywh']
            dx = max(a-(x+w), x-(a+c), 0)
            dy = max(b-(y+h), y-(b+d), 0)
            if math.hypot(dx, dy) <= gap_pixels:
                parents[root(i)] = root(j)
    groups = {}
    for i, region in enumerate(regions):
        groups.setdefault(root(i), []).append(region)
    merged = []
    for members in groups.values():
        area = sum(r['area_pixels'] for r in members)
        if area < min_area:
            continue
        x0 = min(r['bbox_xywh'][0] for r in members)
        y0 = min(r['bbox_xywh'][1] for r in members)
        x1 = max(r['bbox_xywh'][0]+r['bbox_xywh'][2] for r in members)
        y1 = max(r['bbox_xywh'][1]+r['bbox_xywh'][3] for r in members)
        merged.append(dict(area_pixels=area, bbox_xywh=[x0, y0, x1-x0, y1-y0],
            centroid_xy=[sum(r['centroid_xy'][axis]*r['area_pixels'] for r in members)/area for axis in (0, 1)],
            max_temperature_c=max(r['max_temperature_c'] for r in members),
            mean_temperature_c=sum(r['mean_temperature_c']*r['area_pixels'] for r in members)/area,
            component_count=len(members), component_boxes=[r['bbox_xywh'] for r in members]))
    return sorted(merged, key=lambda r: (r['centroid_xy'][0], r['centroid_xy'][1]))


def _iou(first, second):
    x, y, w, h = first
    a, b, c, d = second
    intersection = max(0, min(x+w, a+c)-max(x, a))*max(0, min(y+h, b+d)-max(y, b))
    return intersection / (w*h+c*d-intersection)


def _angle_distance(first, second):
    return abs(math.atan2(math.sin(first-second), math.cos(first-second)))


_DIRECTIONS = {'left_front': '左前方', 'front': '正前方', 'right_front': '右前方'}


def _save_heatmap(path, matrix, regions, roi):
    """PNG in FIR coordinates, NOT an annotation of the unregistered vendor JPG."""
    low, high = float(matrix.min()), float(matrix.max())
    gray = np.rint((matrix-low) / max(high-low, .1) * 255).astype(np.uint8)
    rgb = np.repeat(gray[:, :, None], 3, axis=2)
    def box(x0, y0, x1, y1, color):
        rgb[y0:y0+1, x0:x1] = color
        rgb[y1-1:y1, x0:x1] = color
        rgb[y0:y1, x0:x0+1] = color
        rgb[y0:y1, x1-1:x1] = color
    box(*roi, [0, 160, 255])
    # Tiny bitmap digits keep the deliverable independent of fonts/Pillow/OpenCV.
    digits = ('111101101101111', '010110010010111', '111001111100111',
              '111001111001111', '101101111001001', '111100111001111',
              '111100111101111', '111001001001001', '111101111101111', '111101111001111')
    for index, region in enumerate(regions, 1):
        x, y, w, h = region['bbox_xywh']
        box(x, y, x+w, y+h, [255, 0, 0])
        # Labels index the per-observation target list; stable IDs are in JSON.
        label = str(index)
        ox = min(x, matrix.shape[1]-len(label)*8)
        oy = max(0, y-12)
        rgb[oy:oy+10, ox:ox+len(label)*8] = 0
        for k, digit in enumerate(label):
            for n, bit in enumerate(digits[int(digit)]):
                if bit == '1':
                    px, py = ox+k*8+(n % 3)*2, oy+(n//3)*2
                    rgb[py:py+2, px:px+2] = [255, 255, 0]
    height, width = matrix.shape
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind+data) & 0xffffffff)
    scanlines = b''.join(b'\0'+row.tobytes() for row in rgb)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(scanlines)) + chunk(b'IEND', b''))


class RescueDetector:
    """Create once, call detect once per stopped observation.

    threshold_c/min_area_pixels require field tuning. Detections are heat regions,
    not confirmed people or unique-person counts. Calls on this instance serialize
    via a nonblocking lock: concurrent calls return status=busy.
    """
    def __init__(self, output_dir='p2_results', threshold_c=32.0,
                 min_area_pixels=64, roi=(0, 0, 1, 1),
                 camera_host='192.168.1.214', camera_port=10086,
                 ssh_user='root', ssh_key=None, camera_timeout_s=30,
                 download_timeout_s=20, capture_visible=True,
                 visible_url=None, speak=False, speech_timeout_s=30,
                 audio_device='plughw:CARD=MCP01,DEV=0',
                 assumed_distance_m=None, camera_yaw_offset_rad=0,
                 min_fragment_pixels=8, merge_gap_pixels=20,
                 match_distance_pixels=35, match_iou_threshold=.2,
                 same_view_distance_m=.25, same_view_yaw_deg=10,
                 match_max_age_s=120, horizontal_fov_deg=None,
                 flip_horizontal=False, max_announcements=5):
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.threshold_c = float(threshold_c)
        self.min_area_pixels = int(min_area_pixels)
        self.roi = tuple(float(v) for v in roi)
        if not math.isfinite(self.threshold_c) or self.min_area_pixels < 1:
            raise ValueError('Invalid temperature threshold or minimum area')
        if len(self.roi) != 4 or not (0 <= self.roi[0] < self.roi[2] <= 1 and 0 <= self.roi[1] < self.roi[3] <= 1):
            raise ValueError('ROI must be normalized (left, top, right, bottom)')
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]*', camera_host):
            raise ValueError('Invalid camera host')
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', ssh_user):
            raise ValueError('Invalid SSH user')
        if not 1 <= int(camera_port) <= 65535:
            raise ValueError('Invalid camera port')
        for value in (camera_timeout_s, download_timeout_s, speech_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Timeouts must be finite and positive')
        if assumed_distance_m is not None and (not math.isfinite(assumed_distance_m) or assumed_distance_m <= 0):
            raise ValueError('Assumed distance must be finite and positive')
        if not math.isfinite(camera_yaw_offset_rad):
            raise ValueError('Invalid camera yaw offset')
        for value in (merge_gap_pixels, match_distance_pixels, same_view_distance_m, same_view_yaw_deg):
            if not math.isfinite(value) or value < 0:
                raise ValueError('Merge/matching tolerances must be finite and nonnegative')
        if not 0 < match_iou_threshold <= 1 or not 0 < match_max_age_s < math.inf:
            raise ValueError('Invalid match IoU or maximum age')
        if not 1 <= int(min_fragment_pixels) <= self.min_area_pixels or int(max_announcements) < 1:
            raise ValueError('Invalid fragment area or announcement limit')
        if horizontal_fov_deg is not None and not 0 < horizontal_fov_deg < 180:
            raise ValueError('Horizontal FOV must be between 0 and 180 degrees')
        if assumed_distance_m is not None and horizontal_fov_deg is None:
            raise ValueError('Fixed-distance estimates require horizontal_fov_deg; omit both for direction-only output')
        self.camera_host, self.camera_port = camera_host, int(camera_port)
        self.ssh_user, self.ssh_key = ssh_user, ssh_key
        self.camera_timeout_s, self.download_timeout_s = camera_timeout_s, download_timeout_s
        self.capture_visible = capture_visible
        self.visible_url = visible_url or 'rtsp://{}:8554/visible'.format(camera_host)
        self.speak, self.speech_timeout_s, self.audio_device = speak, speech_timeout_s, audio_device
        self.assumed_distance_m = assumed_distance_m
        self.camera_yaw_offset_rad = camera_yaw_offset_rad
        self.min_fragment_pixels, self.merge_gap_pixels = int(min_fragment_pixels), merge_gap_pixels
        self.match_distance_pixels, self.match_iou_threshold = match_distance_pixels, match_iou_threshold
        self.same_view_distance_m = same_view_distance_m
        self.same_view_yaw_rad = math.radians(same_view_yaw_deg)
        self.match_max_age_s, self.horizontal_fov_deg = match_max_age_s, horizontal_fov_deg
        self.flip_horizontal, self.max_announcements = bool(flip_horizontal), int(max_announcements)
        self._lock = threading.Lock()
        self.session_id = uuid.uuid4().hex
        self._states = {}  # Separate live/replay histories; retained only in this instance.

    def reset(self, checkpoint_id=None):
        """Forget all history, or one checkpoint. No device access or file deletion."""
        if not self._lock.acquire(blocking=False):
            return {'status': 'busy'}
        try:
            if checkpoint_id is None:
                self._states = {}
                self.session_id = uuid.uuid4().hex
            else:
                for state in self._states.values():
                    state['views'] = [v for v in state['views'] if v['checkpoint_id'] != checkpoint_id]
            return {'status': 'ok', 'session_id': self.session_id}
        finally:
            self._lock.release()

    def _select_view(self, state, checkpoint_id, pose, frame_id, now):
        # Anchor pose is fixed: repeated small moves must not accumulate unnoticed.
        signature = [self.roi, self.threshold_c, self.min_area_pixels, self.min_fragment_pixels,
                     self.merge_gap_pixels, self.flip_horizontal, self.camera_yaw_offset_rad,
                     self.horizontal_fov_deg, self.assumed_distance_m]
        compatible = []
        if pose is not None:
            for view in state['views']:
                if (view['checkpoint_id'] != checkpoint_id or view['frame_id'] != frame_id or
                    view['signature'] != signature or view['pose'] is None or
                    now-view['last_seen'] > self.match_max_age_s):
                    continue
                anchor = view['pose']
                distance = math.hypot(pose['x']-anchor['x'], pose['y']-anchor['y'])
                angle = _angle_distance(pose['yaw'], anchor['yaw'])
                if distance <= self.same_view_distance_m and angle <= self.same_view_yaw_rad:
                    compatible.append((distance+angle, view))
        if compatible:
            view = min(compatible, key=lambda item: item[0])[1]
            view['last_seen'] = now
            return view, 'same_view'
        reason = ('pose_missing' if pose is None else
                  'new_or_changed_view' if state['views'] else 'first_view')
        view = dict(view_id='V{:03d}'.format(state['next_view']), checkpoint_id=checkpoint_id,
                    pose=copy.deepcopy(pose), frame_id=frame_id, signature=signature,
                    last_seen=now, tracks=[], next_target=1)
        state['next_view'] += 1
        state['views'].append(view)
        return view, reason

    def _associate(self, view, regions, state, now):
        """Mutually unique matches only. Splits/merges are explicitly ambiguous."""
        old = [t for t in view['tracks'] if now-t['last_seen'] <= self.match_max_age_s]
        edges = []
        for region in regions:
            matches = []
            for index, track in enumerate(old):
                overlap = _iou(region['bbox_xywh'], track['region']['bbox_xywh'])
                distance = math.hypot(*(a-b for a, b in zip(region['centroid_xy'], track['region']['centroid_xy'])))
                if overlap >= self.match_iou_threshold or distance <= self.match_distance_pixels:
                    matches.append(index)
            edges.append(matches)
        uses = {index: sum(index in edge for edge in edges) for index in range(len(old))}
        prior_elsewhere = any(v['tracks'] for v in state['views'] if v is not view)
        stale_local = len(old) < len(view['tracks'])
        targets = []
        for index, (region, matches) in enumerate(zip(regions, edges), 1):
            target = dict(region, observation_index=index, target_id=None,
                          target_number=None, view_id=view['view_id'],
                          is_new_person=None, needs_confirmation=False,
                          possible_target_ids=[old[i]['target_id'] for i in matches])
            if len(matches) == 1 and uses[matches[0]] == 1:
                track = old[matches[0]]
                track.update(region=copy.deepcopy(region), last_seen=now)
                target['association'] = 'matched_in_view'
            elif matches:
                target.update(association='ambiguous_local_match', needs_confirmation=True,
                              confirmation_reason='region_split_or_merge', speech_status='needs_confirmation')
                targets.append(target)
                continue
            else:
                review = prior_elsewhere or stale_local or view['pose'] is None
                reason = ('pose_missing' if view['pose'] is None else
                          'cross_view_identity_unknown' if prior_elsewhere else
                          'previous_match_expired' if stale_local else None)
                number = view['next_target']
                view['next_target'] += 1
                track = dict(target_id='{}-T{:03d}'.format(view['view_id'], number),
                             target_number=number, region=copy.deepcopy(region), last_seen=now,
                             announced=False, needs_confirmation=review, confirmation_reason=reason)
                view['tracks'].append(track)
                target['association'] = 'unconfirmed_observation' if review else 'new_candidate_in_view'
            target.update(target_id=track['target_id'], target_number=track['target_number'],
                          needs_confirmation=track['needs_confirmation'],
                          confirmation_reason=track['confirmation_reason'],
                          speech_status='already_announced' if track['announced'] else 'not_requested')
            targets.append(target)
        return targets

    def _capture(self):
        body = bytes.fromhex('050000') + struct.pack('<ii', 2, 0)
        order = 51000
        frame = b'\xf0\xe0'+struct.pack('<HH', order, len(body))+body
        frame += bytes([functools.reduce(int.__xor__, body, 0)])+b'\xf1\xe1'
        deadline = time.monotonic() + self.camera_timeout_s
        with socket.create_connection((self.camera_host, self.camera_port), timeout=self.camera_timeout_s) as sock:
            def exact(size):
                data = b''
                while len(data) < size:
                    left = deadline-time.monotonic()
                    if left <= 0:
                        raise TimeoutError('Camera response timed out')
                    sock.settimeout(left)
                    part = sock.recv(size-len(data))
                    if not part:
                        raise EOFError('Camera disconnected')
                    data += part
                return data
            sock.settimeout(max(.001, deadline-time.monotonic()))
            sock.sendall(frame)
            header = exact(6)
            seq, size = struct.unpack('<HH', header[2:])
            if header[:2] != b'\xf0\xe0' or seq != order or not 5 <= size <= 65535:
                raise ValueError('Invalid camera response header')
            tail = exact(size+3)
            if tail[:3] != body[:3] or tail[-2:] != b'\xf1\xe1':
                raise ValueError('Invalid camera response command/tail')
            if functools.reduce(int.__xor__, tail[:size], 0) != tail[size]:
                raise ValueError('Camera checksum failed')
            status = int.from_bytes(tail[3:5], 'little')
            if status:
                raise RuntimeError('Camera status {}'.format(status))
            payload = tail[5:size]
            if len(payload) < 5 or struct.unpack_from('<I', payload)[0] != 2:
                raise ValueError('Invalid camera capture mask')
            files = json.loads(payload[4:])
            if not isinstance(files, dict):
                raise ValueError('Invalid camera file reply')
            return files

    def _download(self, remote, target):
        # Normalize the vendor's double slash. Never interpolate unvalidated paths.
        remote = re.sub('/+', '/', str(remote))
        if not re.fullmatch(r'/mnt/sdcard/data/[A-Za-z0-9_-]+/[A-Za-z0-9_-]+\.(fir|jpg)', remote):
            raise ValueError('Invalid camera export path')
        cmd = ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
               '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1']
        if self.ssh_key:
            cmd += ['-i', str(self.ssh_key)]
        cmd += [self.ssh_user+'@'+self.camera_host, 'cat '+remote]
        try:
            with target.open('wb') as output:
                proc = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=output,
                                      stderr=subprocess.PIPE, timeout=self.download_timeout_s)
            if proc.returncode or target.stat().st_size == 0:
                raise RuntimeError('SSH download failed: '+proc.stderr.decode(errors='replace')[-1000:])
        except Exception:
            target.unlink(missing_ok=True)
            raise

    def _visible(self, path):
        cmd = ['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
               '-rtsp_transport', 'tcp', '-i', self.visible_url,
               '-frames:v', '1', '-update', '1', '-y', str(path)]
        proc = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE, timeout=self.download_timeout_s)
        if proc.returncode or not path.exists() or not path.stat().st_size:
            raise RuntimeError('Visible capture failed: '+proc.stderr.decode(errors='replace')[-1000:])

    def _speech(self, text, path):
        # Isolate the vendor-host native TTS library and bound synth/play time.
        cmd = [sys.executable, str(Path(__file__).resolve()), '--speak-text', text,
               '--speech-output', str(path), '--audio-device', self.audio_device]
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, start_new_session=True)
        try:
            _, errors = proc.communicate(timeout=self.speech_timeout_s)
        except subprocess.TimeoutExpired:
            # Kill the aplay child too, so timed-out speech cannot overlap a retry.
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.communicate()
            raise TimeoutError('Speech synthesis/playback timed out')
        if proc.returncode:
            raise RuntimeError('Speech failed: '+errors.decode(errors='replace')[-1000:])

    def detect(self, task_id, robot_pose=None, location_label=None, frame_id='map', fir_path=None,
               checkpoint_id=None):
        """Return JSON-safe dict; input/IO errors => error, target_found=None.

        robot_pose: {x: metres, y: metres, yaw: radians CCW about map +Z}.
        Caller supplies pose at stopped observation. It is NOT verified here.
        checkpoint_id is stable across repeated calls; defaults to task_id.
        History is in-memory only. Never interpret target IDs as unique persons.
        fir_path: offline replay (no camera/network/speech); replay is labeled.
        """
        if not self._lock.acquire(blocking=False):
            return dict(status='busy', target_found=None, error='Detection already running')
        result = dict(status='error', target_found=None, task_id=None,
                      source='offline_replay' if fir_path is not None else 'live_capture',
                      started_at=_now(), session_id=self.session_id, observation_pose=None, frame_id=None,
                      location_valid=False, location_method='observation_only',
                      direction=None, target_type='suspected_heat_source',
                      regions=[], region_count=0, targets=[], candidate_count=0,
                      unique_person_count=None, new_person_count=None,
                      estimated_target_position=None, announcement_complete=None,
                      evidence={}, warnings=[], speech_status='not_requested',
                      timestamp_source='receiver_clock', pose_source='caller_unverified',
                      image_alignment_verified=False, measurement_accuracy_verified=False)
        folder = None
        working = None
        try:
            if not isinstance(task_id, str) or not task_id.strip() or len(task_id) > 128:
                raise ValueError('task_id must contain 1-128 characters')
            if not isinstance(frame_id, str) or not frame_id.strip():
                raise ValueError('frame_id must be a nonempty string')
            if location_label is not None and (not isinstance(location_label, str) or len(location_label) > 100):
                raise ValueError('location_label must be a string of at most 100 characters')
            checkpoint_id = task_id if checkpoint_id is None else checkpoint_id
            if not isinstance(checkpoint_id, str) or not checkpoint_id.strip() or len(checkpoint_id) > 128:
                raise ValueError('checkpoint_id must contain 1-128 characters')
            result.update(task_id=task_id, checkpoint_id=checkpoint_id, frame_id=frame_id,
                          location_label=location_label)
            if robot_pose is not None:
                pose = {key: float(robot_pose[key]) for key in ('x', 'y', 'yaw')}
                if not all(math.isfinite(v) for v in pose.values()):
                    raise ValueError('robot_pose must contain finite x, y, yaw')
                result.update(observation_pose=pose, location_valid=True)
            folder = self.output_dir / (datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex)
            folder.mkdir(parents=True, exist_ok=False)
            result['evidence']['result_json'] = str(folder/'result.json')
            local_fir = folder/'thermal.fir'
            if fir_path is not None:
                shutil.copyfile(fir_path, local_fir)
                result['warnings'].append('Offline replay; supplied pose is replay metadata, not capture-time proof')
            else:
                files = self._capture()
                result['capture_reply_received_at'] = _now()
                _json(folder/'capture_reply.json', files)
                result['evidence']['capture_reply'] = str(folder/'capture_reply.json')
                self._download(files['ir_fir_path'], local_fir)
                try:
                    self._download(files['ir_pic_path'], folder/'thermal_vendor.jpg')
                    result['evidence']['thermal_vendor_image'] = str(folder/'thermal_vendor.jpg')
                except Exception as exc:
                    result['warnings'].append('Vendor image: '+str(exc))
            matrix = read_fir(local_fir)
            if self.flip_horizontal:
                matrix = np.fliplr(matrix)
            result['matrix_orientation'] = 'FIR_horizontal_flip' if self.flip_horizontal else 'FIR_file_order'
            result['data_received_at'] = _now()
            height, width = matrix.shape
            left, top, right, bottom = self.roi
            roi = [int(left*width), int(top*height), int(right*width), int(bottom*height)]
            x0, y0, x1, y1 = roi
            if x1 <= x0 or y1 <= y0:
                raise ValueError('ROI contains no pixels')
            mask = np.zeros(matrix.shape, dtype=bool)
            mask[y0:y1, x0:x1] = matrix[y0:y1, x0:x1] >= self.threshold_c
            components = _regions(mask, matrix, self.min_fragment_pixels)
            regions = _merge_regions(components, self.merge_gap_pixels, self.min_area_pixels)
            now = time.monotonic()
            working = copy.deepcopy(self._states.get(result['source'], {'views': [], 'next_view': 1}))
            view, view_reason = self._select_view(working, checkpoint_id, result['observation_pose'], frame_id, now)
            targets = self._associate(view, regions, working, now)
            result.update(view_id=view['view_id'], view_relation=view_reason, targets=targets,
                          candidate_count=len(targets),
                          new_candidate_count_in_view=sum(t['association'] == 'new_candidate_in_view' for t in targets),
                          unconfirmed_observation_count=sum(t['association'] == 'unconfirmed_observation' for t in targets),
                          matched_candidate_count=sum(t['association'] == 'matched_in_view' for t in targets),
                          needs_confirmation=any(t['needs_confirmation'] for t in targets))
            result['parameters'] = dict(threshold_c=self.threshold_c, min_area_pixels=self.min_area_pixels,
                                        roi_normalized=list(self.roi), roi_pixels=roi,
                                        assumed_distance_m=self.assumed_distance_m,
                                        camera_yaw_offset_rad=self.camera_yaw_offset_rad,
                                        min_fragment_pixels=self.min_fragment_pixels,
                                        merge_gap_pixels=self.merge_gap_pixels,
                                        match_distance_pixels=self.match_distance_pixels,
                                        match_iou_threshold=self.match_iou_threshold,
                                        same_view_distance_m=self.same_view_distance_m,
                                        same_view_yaw_deg=math.degrees(self.same_view_yaw_rad),
                                        match_max_age_s=self.match_max_age_s,
                                        horizontal_fov_deg=self.horizontal_fov_deg,
                                        flip_horizontal=self.flip_horizontal,
                                        max_announcements=self.max_announcements)
            result['temperature_statistics'] = dict(min_c=float(matrix.min()), max_c=float(matrix.max()),
                                                     mean_c=float(matrix.mean()))
            np.savetxt(folder/'temperature.csv', matrix, delimiter=',', fmt='%.1f')
            _save_heatmap(folder/'thermal_detection.png', matrix, regions, roi)
            result['evidence'].update(temperature_fir=str(local_fir), temperature_csv=str(folder/'temperature.csv'),
                                      thermal_image=str(folder/'thermal_detection.png'))
            if fir_path is None and self.capture_visible:
                try:
                    self._visible(folder/'visible.jpg')
                    result['evidence']['visible_image'] = str(folder/'visible.jpg')
                    result['visible_received_at'] = _now()
                except Exception as exc:
                    result['warnings'].append('Visible image: '+str(exc))
            found = bool(targets)
            pose = result['observation_pose']
            where = location_label or checkpoint_id
            if pose is not None:
                where += '，观测位置横坐标{:.1f}米，纵坐标{:.1f}米'.format(pose['x'], pose['y'])
            announced_this_call = 0
            for target in targets:
                u = target['centroid_xy'][0]
                direction = 'left_front' if u < width/3 else 'right_front' if u >= 2*width/3 else 'front'
                target.update(direction=direction, direction_text=_DIRECTIONS[direction],
                              observation_pose=copy.deepcopy(pose), frame_id=frame_id,
                              estimated_target_position=None)
                if pose is not None and self.assumed_distance_m is not None:
                    focal = width / (2*math.tan(math.radians(self.horizontal_fov_deg)/2))
                    bearing = math.atan(((width-1)/2-u)/focal)
                    angle = pose['yaw']+self.camera_yaw_offset_rad+bearing
                    target['estimated_target_position'] = dict(
                        x=pose['x']+self.assumed_distance_m*math.cos(angle),
                        y=pose['y']+self.assumed_distance_m*math.sin(angle), frame_id=frame_id,
                        method='assumed_range_configured_fov', distance_measured=False,
                        bearing_rad=bearing, fov_calibration_verified=False)
                if target['association'] == 'ambiguous_local_match':
                    target['speech_text'] = where+'，'+_DIRECTIONS[direction]+'热源与已有目标对应关系待确认。'
                    continue  # Do not assign a new identity or replay an ambiguous target.
                message = where+'，目标{}，{}发现疑似人员热源。'.format(target['target_number'], _DIRECTIONS[direction])
                if target['needs_confirmation']:
                    message += '是否与此前目标重复待确认。'
                target['speech_text'] = message
                if fir_path is not None:
                    target['speech_status'] = 'disabled_for_replay'
                elif target['speech_status'] == 'already_announced' or not self.speak:
                    continue
                elif announced_this_call >= self.max_announcements:
                    target['speech_status'] = 'deferred'
                else:
                    announced_this_call += 1
                    try:
                        speech_path = folder/(target['target_id']+'_speech.wav')
                        self._speech(message, speech_path)
                        target['speech_status'] = 'success'
                        target['speech_wav'] = str(speech_path)
                        track = next(t for t in view['tracks'] if t['target_id'] == target['target_id'])
                        track['announced'] = True
                    except Exception as exc:
                        target['speech_status'] = 'failed'
                        result['warnings'].append(target['target_id']+': '+str(exc))
            statuses = [t['speech_status'] for t in targets]
            if fir_path is not None:
                speech_status = 'disabled_for_replay'
            elif not found:
                speech_status = 'not_needed'
            elif all(s == 'already_announced' for s in statuses):
                speech_status = 'suppressed_repeated'
            elif all(s in ('success', 'already_announced') for s in statuses):
                speech_status = 'success'
            elif 'success' in statuses or 'already_announced' in statuses:
                speech_status = 'partial'
            elif 'failed' in statuses:
                speech_status = 'failed'
            elif 'needs_confirmation' in statuses:
                speech_status = 'needs_confirmation'
            else:
                speech_status = 'not_requested'
            result.update(status='ok', target_found=found, regions=regions, region_count=len(regions),
                          direction=(targets[0]['direction'] if len(targets) == 1 else
                                     'multiple' if targets else None),
                          speech_status=speech_status,
                          announcement_complete=(all(s in ('success', 'already_announced') for s in statuses)
                                                 if found and fir_path is None else None),
                          speech_text=' '.join(t['speech_text'] for t in targets) if found else
                          where+'，当前检测区域未发现符合条件的热源。')
            # Backward-compatible convenience field only for a single candidate.
            if len(targets) == 1:
                result['estimated_target_position'] = targets[0]['estimated_target_position']
        except Exception as exc:
            result.update(status='error', target_found=None, error=str(exc), error_type=type(exc).__name__)
        finally:
            result['finished_at'] = _now()
            try:
                if folder is not None:
                    _json(folder/'result.json', result)
                    if result['status'] == 'ok' and working is not None:
                        self._states[result['source']] = working
            except Exception as exc:
                result.update(status='error', target_found=None, error='Result persistence failed: '+str(exc))
            finally:
                self._lock.release()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fir', type=Path, help='Offline FIR replay; never contacts devices or plays speech')
    parser.add_argument('--task-id', default='p2_observation')
    parser.add_argument('--location-label')
    parser.add_argument('--checkpoint-id', help='Stable observation point ID; defaults to task-id')
    parser.add_argument('--pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW_RAD'))
    parser.add_argument('--frame-id', default='map')
    parser.add_argument('--output-dir', default='p2_results')
    parser.add_argument('--threshold', type=float, default=32)
    parser.add_argument('--min-area', type=int, default=64)
    parser.add_argument('--roi', nargs=4, type=float, default=(0, 0, 1, 1))
    parser.add_argument('--camera-host', default='192.168.1.214')
    parser.add_argument('--camera-port', type=int, default=10086)
    parser.add_argument('--ssh-user', default='root')
    parser.add_argument('--ssh-key')
    parser.add_argument('--no-visible', action='store_true')
    parser.add_argument('--speak', action='store_true', help='Play only on a positive LIVE detection')
    parser.add_argument('--assumed-distance', type=float)
    parser.add_argument('--camera-yaw-offset', type=float, default=0)
    parser.add_argument('--horizontal-fov', type=float)
    parser.add_argument('--merge-gap', type=float, default=20)
    parser.add_argument('--min-fragment', type=int, default=8)
    parser.add_argument('--flip-horizontal', action='store_true')
    parser.add_argument('--audio-device', default='plughw:CARD=MCP01,DEV=0')
    parser.add_argument('--speak-text', help=argparse.SUPPRESS)
    parser.add_argument('--speech-output', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.speak_text is not None:
        if not args.speech_output or not args.speak_text.strip() or len(args.speak_text) > 500:
            parser.error('Invalid speech arguments')
        speech = synthesize(args.speak_text, args.speech_output)
        subprocess.run(['aplay', '-D', args.audio_device, str(args.speech_output)],
                       stdin=subprocess.DEVNULL, check=True, timeout=speech['duration_seconds']+10)
        return
    detector = RescueDetector(output_dir=args.output_dir, threshold_c=args.threshold,
        min_area_pixels=args.min_area, roi=args.roi, camera_host=args.camera_host,
        camera_port=args.camera_port, ssh_user=args.ssh_user, ssh_key=args.ssh_key,
        capture_visible=not args.no_visible, speak=args.speak, audio_device=args.audio_device,
        assumed_distance_m=args.assumed_distance, camera_yaw_offset_rad=args.camera_yaw_offset,
        horizontal_fov_deg=args.horizontal_fov, merge_gap_pixels=args.merge_gap,
        min_fragment_pixels=args.min_fragment, flip_horizontal=args.flip_horizontal)
    pose = dict(zip(('x', 'y', 'yaw'), args.pose)) if args.pose is not None else None
    result = detector.detect(args.task_id, pose, args.location_label, args.frame_id, args.fir, args.checkpoint_id)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result['status'] == 'ok' else 1


# Reused from the locally verified 2026-10-12 speech test.
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
