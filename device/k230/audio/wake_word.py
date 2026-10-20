"""Headless open_kws enrollment and detection. No UART or motion interfaces."""
import gc
import json
import os
import time
import open_kws
from media.pyaudio import PyAudio, paInt16

ROOT = '/sdcard/examples/31-OPEN-KWS/'
CHUNK = 2560


def engine():
    eng = open_kws.Engine(ROOT + 'model/kws.kmodel', ROOT + 'assets')
    eng.set_quality(clip_ratio=0.005, min_snr=10.0)
    eng.set_sensitivity('normal')
    eng.set_verify(n_hyps=8, alpha=2.0, delta=1.25)
    return eng


def make_folder(path):
    current = ''
    for part in path.strip('/').split('/'):
        current += '/' + part
        try:
            os.mkdir(current)
        except OSError:
            os.stat(current)


def capture_take(folder, number):
    make_folder(folder)
    eng, audio, stream = None, None, None
    try:
        eng = engine()
        eng.enroll_begin()
        audio = PyAudio()
        stream = audio.open(format=paInt16, channels=1, rate=16000,
                            input=True, frames_per_buffer=CHUNK)
        stream.volume(vol=100)
        print('VOICE_WARMUP')
        # Discard the observed microphone power-on DC transient before VAD.
        for _ in range(32):
            os.exitpoint()
            stream.read()
        noise_levels = []
        for _ in range(10):
            os.exitpoint()
            pcm = stream.read()
            noise_levels.append(max(open_kws.level(pcm)[0], 0.0001))
        noise = min(noise_levels)
        print('VOICE_READY ' + str(number))
        chunks, started, silence = [], False, 0
        for chunk_index in range(63 + 25):
            os.exitpoint()
            pcm = stream.read()
            if len(pcm) != CHUNK * 2:
                raise RuntimeError('Incomplete audio chunk')
            rms, peak = open_kws.level(pcm)
            chunks.append(pcm)
            if not started:
                if rms > max(noise * 2.5, 0.004) or rms > 0.03:
                    started = True
                    print('VOICE_SPEECH')
                else:
                    if len(chunks) > 3:
                        chunks.pop(0)
                    if chunk_index >= 62:
                        break
                    continue
            silence = silence + 1 if rms < max(noise * 1.8, 0.003) else 0
            if silence >= 4 or len(chunks) >= 25:
                break
        if not started:
            print('VOICE_TAKE ' + json.dumps({'ok': False, 'reason': 'no speech'}))
            return
        pcm = b''.join(chunks)
        span = open_kws.trim(pcm)
        segment = pcm[span[0]*2:span[1]*2] if span else pcm
        ok, reason, metrics = eng.enroll_add(segment, noise)
        result = {'ok': ok, 'reason': reason, 'metrics': metrics, 'noise': noise,
                  'number': number, 'mic_volume': 100}
        if ok:
            with open(folder + '/take_' + str(number) + '.pcm', 'wb') as f:
                f.write(segment)
            with open(folder + '/take_' + str(number) + '.json', 'w') as f:
                json.dump(result, f)
        print('VOICE_TAKE ' + json.dumps(result))
    finally:
        if stream is not None:
            stream.stop_stream()
            stream.close()
        if audio is not None:
            audio.terminate()
        if eng is not None:
            eng.enroll_end()
            eng.deinit()
        gc.collect()


def finalize(folder):
    eng = engine()
    try:
        eng.enroll_begin()
        for number in (1, 2, 3):
            prefix = folder + '/take_' + str(number)
            with open(prefix + '.json') as f:
                take = json.load(f)
            with open(prefix + '.pcm', 'rb') as f:
                pcm = f.read()
            ok, reason, metrics = eng.enroll_add(pcm, take['noise'])
            if not ok:
                raise ValueError('Take %d rejected: %s' % (number, reason))
        began = time.ticks_ms()
        while True:
            os.exitpoint()
            done, progress, total = eng.enroll_step()
            print('VOICE_PROGRESS %d/%d' % (progress, total))
            if done:
                break
            if time.ticks_diff(time.ticks_ms(), began) > 120000:
                raise RuntimeError('Enrollment computation timed out')
        rec = eng.enroll_result('sida')
        eng.enroll_end()
        rec['label'] = '思达思达'
        rec['mic_volume'] = 100
        eng.add_keyword(rec)
        path = folder + '/keyword.json'
        with open(path + '.tmp', 'w') as f:
            json.dump(rec, f)
        os.rename(path + '.tmp', path)
        print('VOICE_ENROLLED ' + json.dumps({'path': path, 'keyword': rec}))
    finally:
        eng.enroll_end()
        eng.deinit()
        gc.collect()


def listen(path, seconds):
    eng, audio, stream = None, None, None
    try:
        with open(path) as f:
            rec = json.load(f)
        eng = engine()
        eng.add_keyword(rec)
        audio = PyAudio()
        stream = audio.open(format=paInt16, channels=1, rate=16000,
                            input=True, frames_per_buffer=CHUNK)
        stream.volume(vol=rec.get('mic_volume', 100))
        for _ in range(32):
            os.exitpoint()
            stream.read()
        eng.reset()
        print('VOICE_LISTEN_READY')
        count = 0
        for _ in range(int(seconds * 16000 / CHUNK)):
            os.exitpoint()
            for event in eng.push(stream.read()):
                count += 1
                print('VOICE_WAKE ' + json.dumps(event))
        print('VOICE_LISTEN_DONE ' + str(count))
    finally:
        if stream is not None:
            stream.stop_stream()
            stream.close()
        if audio is not None:
            audio.terminate()
        if eng is not None:
            eng.deinit()
        gc.collect()
