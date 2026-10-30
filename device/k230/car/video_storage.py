"""Recording capacity checks, shared by the transient K230 script and tests."""


def required_free_bytes(duration_ms, bitrate_kbps=4096):
    if duration_ms <= 0 or bitrate_kbps <= 0:
        raise ValueError('Duration and bitrate must be positive')
    nominal = (duration_ms * bitrate_kbps + 7) // 8
    # Encoder rate may vary. Leave 25% headroom and 8 MiB for metadata/filesystem.
    return (nominal * 5 + 3) // 4 + 8 * 1024 * 1024


def recording_space(statvfs, duration_ms, bitrate_kbps=4096):
    block_bytes = statvfs[1] or statvfs[0]
    free_bytes = block_bytes * statvfs[4]
    required = required_free_bytes(duration_ms, bitrate_kbps)
    return {'available_bytes': free_bytes, 'required_bytes': required,
            'sufficient': free_bytes >= required}


def write_video_packet(stream, payload):
    remaining = memoryview(payload)
    while remaining:
        written = stream.write(remaining)
        if not isinstance(written, int) or not 0 < written <= len(remaining):
            raise OSError('SD video write failed: no progress; check free space/card')
        remaining = remaining[written:]
