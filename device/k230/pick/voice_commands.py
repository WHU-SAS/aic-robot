"""Registered word IDs map to explicit actions; independent of hardware."""


def registered_commands(config):
    commands = [{'keyword_id': config.get('keyword_id', 'sida'),
                 'word': config['wake_word'], 'action': 'pick',
                 'enrollment_file': config['enrollment_file']}]
    for command in config.get('commands', {}).values():
        if command.get('enrollment_file'):
            commands.append(command)
    ids = []
    for command in commands:
        key = command['keyword_id']
        if not key or key in ids:
            raise ValueError('Missing or duplicate keyword ID')
        if command['action'] not in ('pick', 'ready'):
            raise ValueError('Unknown voice action')
        if not command['enrollment_file']:
            raise ValueError('Missing keyword registration')
        ids.append(key)
    return commands


def resolve_events(events, commands):
    # open_kws returns (kw_id, score, threshold, start_ms, end_ms).
    matches = []
    for event in events:
        if not isinstance(event, (list, tuple)) or not event:
            continue
        for command in commands:
            if event[0] == command['keyword_id'] and command not in matches:
                matches.append(command)
    # Two different commands in one audio chunk are ambiguous: do neither.
    return matches[0] if len(matches) == 1 else None
