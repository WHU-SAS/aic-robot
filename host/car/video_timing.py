"""Video timing from recorded observations; no camera or codec imports."""
import math
import statistics


def frame_times(rows, fps):
    if not rows or not math.isfinite(fps) or fps <= 0:
        raise ValueError('Video needs frames and a positive output FPS')
    times = [0.]
    periods = []
    for previous, row in zip(rows, rows[1:]):
        period = row.get('frame_period_ms')
        if not isinstance(period, (int, float)) or not math.isfinite(period) or period <= 0:
            period = row['capture_timestamp_ms'] - previous['capture_timestamp_ms']
        if not isinstance(period, (int, float)) or not math.isfinite(period) or period <= 0:
            raise ValueError('Invalid recording timestamps/periods')
        periods.append(period / 1000)
        times.append(times[-1] + period / 1000)
    tail = statistics.median(periods) if periods else 1 / fps
    return times, times[-1] + tail
