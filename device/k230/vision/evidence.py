"""Small, durable raw-image records; no camera, motor or retention policy.

Ticks identify a local run, not capture time or UTC. Callers supply frame metadata
from the actual capture and report any OSError; this module never drops old runs.
"""
import os
import json
import time
try:
    import uhashlib as hashlib
    import ubinascii as binascii
except ImportError:
    import hashlib
    import binascii

SCHEMA = 'aic.vision_evidence.v1'


def _errno(error):
    return getattr(error, 'errno', None) or (error.args[0] if error.args else None)


def _exists(path):
    try:
        os.stat(path)
        return True
    except OSError as error:
        if _errno(error) == 2:
            return False
        raise


def _mkdirs(path):
    current = '/' if path.startswith('/') else ''
    for part in path.split('/'):
        if not part or part == '.':
            continue
        current = current.rstrip('/') + '/' + part if current else part
        try:
            os.mkdir(current)
        except OSError as error:
            if _errno(error) != 17:
                raise
            if not os.stat(current)[0] & 0x4000:
                raise ValueError('Evidence directory is a file: ' + current)


def _name(value):
    if not isinstance(value, str) or not value:
        raise ValueError('Expected a non-empty evidence task/label')
    for char in value:
        if not ('a' <= char <= 'z' or 'A' <= char <= 'Z' or
                '0' <= char <= '9' or char in '_-'):
            raise ValueError('Evidence task/label permits only ASCII letters, digits, _ and -')
    return value


def _snapshot(value):
    return json.loads(json.dumps(value))


def _ticks():
    clock = getattr(time, 'ticks_ms', None)
    return clock() if clock else int(time.monotonic() * 1000)


def _write_all(stream, data):
    # Slicing a memoryview avoids copying/re-encoding an owned JPEG on retries.
    data = data if isinstance(data, str) else memoryview(data)
    offset = 0
    while offset < len(data):
        count = stream.write(data[offset:])
        if count is None or count <= 0 or count > len(data) - offset:
            raise OSError(5, 'Evidence write made no valid progress')
        offset += count


def _atomic_json(path, value):
    temporary = path + '.tmp'
    backup = path + '.replace_previous'
    if _exists(backup):
        # An interrupted replacement requires recovery; never overwrite its
        # only surviving old metadata or guess which version should win.
        raise OSError(17, 'Unresolved evidence replacement backup: ' + backup)
    # CanMV text writes count UTF-8 bytes, not Python string characters.
    # Encode once and use the same byte-progress contract as owned JPEG data.
    payload = (json.dumps(value) + '\n').encode('utf-8')
    with open(temporary, 'wb') as stream:
        _write_all(stream, payload)
        stream.flush()
    try:
        os.rename(temporary, path)
    except OSError as error:
        if _errno(error) != 17:
            raise
        # CanMV FAT rejects renaming onto a file. Preserve the prior committed
        # manifest until the replacement is installed, restoring it on error.
        os.rename(path, backup)
        try:
            os.rename(temporary, path)
        except BaseException:
            os.rename(backup, path)
            raise
        os.remove(backup)


def _file_digest(path):
    digest, total = hashlib.sha256(), 0
    with open(path, 'rb') as stream:
        while True:
            block = stream.read(4096)
            if not block:
                break
            digest.update(block)
            total += len(block)
    return binascii.hexlify(digest.digest()).decode('ascii'), total


class EvidenceRun:
    """One immutable run directory, with atomically replaced progress metadata.

    new_frame returns a reserved path for the caller's existing JPEG save. It
    persists metadata first, so an interrupted save remains discoverable. Keep
    the run's writer single-owner. finish hashes bytes that actually reached the
    filesystem; a missing/empty image makes the archive partial, never successful.
    """
    def __init__(self, root, task, config, version=None):
        task = _name(task)
        root = str(root).rstrip('/') or '/'
        config, version = _snapshot(config), _snapshot(version)
        _mkdirs(root)
        created = _ticks()
        attempt = 0
        while True:
            run_name = '%s_%s_%s' % (task, created, attempt)
            directory = root.rstrip('/') + '/' + run_name
            try:
                os.mkdir(directory)
                break
            except OSError as error:
                if _errno(error) != 17:
                    raise
                attempt += 1
        self.directory = directory
        self.path = directory
        self.manifest_path = directory + '/manifest.json'
        self._closed = False
        self.manifest = {
            'schema': SCHEMA, 'task': task,
            'created_ticks_ms': created,
            'created_time_source': 'local_ticks_not_capture_time_or_utc',
            'config_file': 'config.json', 'version': version,
            'frames': [], 'status': 'recording', 'error': None,
            'archive_complete': False,
        }
        _atomic_json(directory + '/config.json', config)
        self._persist()

    def _persist(self):
        _atomic_json(self.manifest_path, self.manifest)

    def _writable(self):
        if self._closed:
            raise ValueError('Evidence run is already finalized')

    def _entry(self, label):
        for entry in self.manifest['frames']:
            if entry['label'] == label:
                return entry
        raise ValueError('Unknown evidence frame: ' + str(label))

    def new_frame(self, label, frame):
        return self._reserve_frame(label, frame)

    def _reserve_frame(self, label, frame, expected_bytes=None):
        self._writable()
        label = _name(label)
        if any(entry['label'] == label for entry in self.manifest['frames']):
            raise ValueError('Evidence label already reserved: ' + label)
        image_file = label + '.jpg'
        path = self.directory + '/' + image_file
        if _exists(path):
            raise ValueError('Refusing to overwrite evidence image: ' + path)
        self.manifest['frames'].append({
            'label': label, 'image_file': image_file,
            'frame': _snapshot(frame), 'observation': None,
            'image_sha256': None, 'image_bytes': None,
            'image_expected_bytes': expected_bytes,
            'image_status': 'pending',
        })
        self._persist()
        return path

    def record_observation(self, label, observation):
        self._writable()
        self._entry(label)['observation'] = _snapshot(observation)
        self._persist()

    def save_jpeg(self, label, jpeg, frame, observation=None):
        """Write already-owned JPEG bytes; no recompression or image conversion."""
        path = self._reserve_frame(label, frame, len(jpeg))
        # A caller may reserve a path for native image.save instead. This helper
        # alone owns this newly reserved path and still refuses existing files.
        if _exists(path):
            raise ValueError('Refusing to overwrite evidence image: ' + path)
        with open(path, 'wb') as stream:
            _write_all(stream, jpeg)
            stream.flush()
        self.record_observation(label, observation)
        return path

    def finish(self, status, error=None):
        self._writable()
        complete = True
        for entry in self.manifest['frames']:
            path = self.directory + '/' + entry['image_file']
            if not _exists(path):
                entry['image_status'] = 'missing'
                complete = False
                continue
            digest, size = _file_digest(path)
            entry['image_sha256'], entry['image_bytes'] = digest, size
            expected = entry.get('image_expected_bytes')
            entry['image_status'] = ('empty' if not size else
                                     'incomplete' if expected is not None and size != expected else
                                     'complete')
            if entry['image_status'] != 'complete':
                complete = False
        self.manifest['requested_status'] = status
        self.manifest['status'] = status if complete else 'partial'
        self.manifest['error'] = None if error is None else str(error)
        self.manifest['archive_complete'] = complete
        self._persist()
        self._closed = True
        return self.manifest
