"""Bounded microphone check for CanMV v1.8. No UART or motion commands."""
import gc
import json
import math
import os
import struct
from media.pyaudio import PyAudio, paInt16


def run_microphone_probe(seconds=3, remote_wav=None):
    audio = PyAudio()
    stream = None
    frames = []
    blocks = []
    rate, chunk = 16000, 1600
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        stream = audio.open(format=paInt16, channels=1, rate=rate,
                            input=True, frames_per_buffer=chunk)
        # Initial probe showed a decaying DC transient for about one second.
        for _ in range(15):
            os.exitpoint()
            stream.read()
        print('MIC_READY')
        for index in range(int(seconds * rate / chunk)):
            os.exitpoint()
            pcm = stream.read()
            if len(pcm) != chunk * 2:
                raise RuntimeError('Short PCM block: ' + str(len(pcm)))
            values = struct.unpack('<' + str(chunk) + 'h', pcm)
            dc = sum(values) / chunk
            rms = math.sqrt(max(0, sum(v * v for v in values) / chunk - dc * dc))
            peak = max(abs(min(values)), abs(max(values)))
            clipped = sum(1 for v in values if abs(v) >= 32760)
            blocks.append({'block': index, 'rms': rms, 'peak': peak,
                           'dc': dc, 'clipped_samples': clipped})
            if remote_wav:
                frames.append(pcm)
        if remote_wav:
            parent = remote_wav.rsplit('/', 1)[0]
            try:
                os.mkdir(parent)
            except OSError:
                os.stat(parent)
            # A simple fixed-length PCM WAV avoids dependencies on wave writer APIs.
            pcm = b''.join(frames)
            header = (b'RIFF' + struct.pack('<I', 36 + len(pcm)) + b'WAVEfmt ' +
                      struct.pack('<IHHIIHH', 16, 1, 1, rate, rate * 2, 2, 16) +
                      b'data' + struct.pack('<I', len(pcm)))
            with open(remote_wav, 'wb') as output:
                output.write(header)
                output.write(pcm)
        print('MIC_RESULT ' + json.dumps({'sample_rate': rate, 'channels': 1,
              'sample_bits': 16, 'seconds': seconds, 'blocks': blocks,
              'remote_wav': remote_wav}))
        print('MIC_PROBE_PASS')
    finally:
        if stream is not None:
            stream.stop_stream()
            stream.close()
        audio.terminate()
        frames = []
        gc.collect()
