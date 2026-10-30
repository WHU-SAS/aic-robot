"""Standalone '思达思达' -> one distance or angle run. No camera or arm calls."""
import gc
import json
import os
import time


def wait_for_wake(voice, warmup_chunks):
    from wake_word import engine, CHUNK
    from media.pyaudio import PyAudio, paInt16
    with open(voice['enrollment_file']) as f:
        keyword = json.load(f)
    eng = audio = stream = None
    try:
        eng = engine(voice['detection_sensitivity'])
        eng.add_keyword(keyword)
        audio = PyAudio()
        stream = audio.open(format=paInt16, channels=1, rate=16000,
                            input=True, frames_per_buffer=CHUNK)
        stream.volume(vol=keyword.get('mic_volume', 100))
        for unused in range(warmup_chunks):
            os.exitpoint()
            stream.read()
        eng.reset()
        print('CAR_STRAIGHT voice_ready 思达思达')
        while True:
            os.exitpoint()
            events = eng.push(stream.read())
            if events and any(event[0] == keyword['kw_id'] for event in events):
                return
    finally:
        if stream is not None:
            stream.stop_stream()
            stream.close()
        if audio is not None:
            audio.terminate()
        if eng is not None:
            eng.deinit()
        gc.collect()


def wheel_distances(state, baseline, mm_per_count):
    def delta(value, base):
        return ((value - base + 2147483648) % 4294967296) - 2147483648
    return [delta(state['encoder_a'], baseline['encoder_a']) * mm_per_count,
            -delta(state['encoder_b'], baseline['encoder_b']) * mm_per_count]


def run_motion(link, config, baseline=None):
    turning = config.get('action', 'move') == 'turn'
    baseline = link.sync() if baseline is None else baseline
    states = [link.turn(config['angle_deg']) if turning else link.move(config['distance_mm'])]
    while states[-1]['run']:
        time.sleep_ms(config['heartbeat_interval_ms'])
        states.append(link.expect('HB'))
    cutoff = states[-1]
    stable_at = cutoff['ms']
    counts = (cutoff['encoder_a'], cutoff['encoder_b'])
    while ((states[-1]['ms'] - stable_at) & 0xFFFFFFFF) < config['stopped_observation_ms']:
        time.sleep_ms(50)
        state = link.expect('STATUS')
        new_counts = (state['encoder_a'], state['encoder_b'])
        if new_counts != counts:
            stable_at, counts = state['ms'], new_counts
        states.append(state)
    wheel_mm = wheel_distances(states[-1], baseline, config['mm_per_count'])
    cutoff_mm = wheel_distances(cutoff, baseline, config['mm_per_count'])
    result = {'action': 'turn' if turning else 'move',
              'baseline': baseline, 'states': states, 'wheel_nominal_mm': wheel_mm,
              'mean_nominal_mm': sum(wheel_mm) / 2,
              'cutoff_nominal_mm': sum(cutoff_mm) / 2,
              'stop_reason': cutoff['reason'], 'physical_distance_measured': False}
    if turning:
        degrees_per_mm = 180 / (3.141592653589793 * config['track_width_mm'])
        result.update(requested_angle_deg=config['angle_deg'],
                      nominal_angle_deg=(wheel_mm[1] - wheel_mm[0]) * degrees_per_mm,
                      cutoff_nominal_angle_deg=(cutoff_mm[1] - cutoff_mm[0]) * degrees_per_mm,
                      track_width_mm=config['track_width_mm'], physical_angle_measured=False)
    else:
        result.update(requested_mm=config['distance_mm'],
                      allowed_error_fraction=config['allowed_error_fraction'])
    return result


def run_forever(root):
    from motor_link import MotorLink
    with open(root + '/config.json') as f:
        config = json.load(f)
    print('CAR_STRAIGHT configured ' + json.dumps(config))
    warmup = 32
    while True:
        wait_for_wake(config['voice'], warmup)
        warmup = 0
        link = MotorLink()
        try:
            result = run_motion(link, config)
        finally:
            link.close()
        # All SD writes happen after the one continuous drive and its coast.
        with open('/sdcard/aic/car_straight/runs.jsonl', 'a') as f:
            f.write(json.dumps(result) + '\n')
        print('CAR_STRAIGHT complete ' + json.dumps({
            'action': result['action'], 'nominal_mm': result['mean_nominal_mm'],
            'nominal_angle_deg': result.get('nominal_angle_deg'), 'reason': result['stop_reason']}))
        gc.collect()
