"""Offline Chinese TTS on the navigation host; --play plays through MCP01.

Uses the existing libespeak-ng library, not the manufacturer's business API.
API: https://github.com/espeak-ng/espeak-ng/blob/master/src/include/espeak-ng/speak_lib.h
"""
import argparse
import ctypes as c
import ctypes.util
import json
from pathlib import Path
import subprocess
import wave


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--text', required=True)
    parser.add_argument('--output', type=Path, default=Path('speech.wav'))
    parser.add_argument('--play', action='store_true')
    parser.add_argument('--device', default='plughw:CARD=MCP01,DEV=0')
    args = parser.parse_args()
    if not args.text.strip() or len(args.text) > 500:
        parser.error('Text must contain 1–500 characters')
    result = synthesize(args.text, args.output)
    result['playback'] = 'not_requested'
    if args.play:
        played = subprocess.run(['aplay', '-D', args.device, str(args.output)],
                                timeout=result['duration_seconds']+15)
        result['playback'] = 'success' if played.returncode == 0 else 'failed'
        result['playback_returncode'] = played.returncode
    args.output.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result['playback'] == 'failed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
