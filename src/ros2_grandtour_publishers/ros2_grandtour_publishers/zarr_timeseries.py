"""Zarr-v2 mission stream reader for the GrandTour dataset layout.

Every downloaded stream lives at ``<mission_dir>/data/<name>/<name>`` as a
zarr group (``zarr_format: 2``, blosc/lz4 compressed) with a ``timestamp``
1-D array (float64 Unix epoch seconds) and one or more per-sample fields of
shape ``(num_samples, ...)``. Chunking is along axis 0, so a naive per-row
``array[i]`` re-decompresses the whole containing chunk on every call
(~130 ms for the lidar ``points`` array on this machine). ``ChunkCache``
amortizes that by keeping the last decompressed chunk per field in memory,
which is enough for the strictly increasing row access pattern used during
playback.
"""

from pathlib import Path

import numpy as np
import zarr


class ChunkCache:
    """Cache the most recently decompressed chunk of a 1-D-chunked array."""

    def __init__(self, array):
        self._array = array
        self._chunk_rows = array.chunks[0]
        self._start = None
        self._block = None

    def row(self, index):
        start = (index // self._chunk_rows) * self._chunk_rows
        if start != self._start:
            self._block = self._array[start:start + self._chunk_rows]
            self._start = start
        return self._block[index - start]


class StreamReader:
    """One resolved GrandTour stream: its zarr group plus lazy field access."""

    def __init__(self, name, group):
        self.name = name
        self.group = group
        self.attrs = dict(group.attrs)
        self.frame_id = self.attrs.get('frame_id', name)
        self.timestamps = np.asarray(group['timestamp'][:], dtype=np.float64)
        self._caches = {}

    def __len__(self):
        return self.timestamps.shape[0]

    def field(self, name):
        return self.group[name]

    def row(self, field_name, index):
        cache = self._caches.get(field_name)
        if cache is None:
            cache = ChunkCache(self.group[field_name])
            self._caches[field_name] = cache
        return cache.row(index)

    def has_field(self, name):
        return name in self.group


def resolve_stream(mission_dir, candidates):
    """Return the first candidate present under ``<mission_dir>/data``.

    Mirrors the fallback-chain resolution in ``tools/download_grandtour.py``:
    stream availability and naming vary per mission, so callers pass a
    preference-ordered candidate list and the first one found on disk wins.
    """
    data_dir = Path(mission_dir) / 'data'
    for candidate in candidates:
        group_path = data_dir / candidate / candidate
        if (group_path / '.zgroup').is_file():
            group = zarr.open_group(str(group_path), mode='r')
            # A few derived GrandTour archives are only point-array overlays
            # (ETH-1's *_filtered lidar is one): without a timestamp array
            # they are not independently replayable. Treat them as an
            # incomplete candidate and continue through the fallback chain.
            if 'timestamp' not in group:
                continue
            return StreamReader(candidate, group)
    return None


def load_static_tf_tree(mission_dir):
    """Parse ``metadata/tf.yaml``: the full static extrinsics tree.

    Returns a dict of ``child_frame_id -> {base_frame_id, translation, rotation}``,
    where ``rotation`` is ``(x, y, z, w)`` and ``translation`` is ``(x, y, z)``.
    """
    import yaml

    yaml_path = Path(mission_dir) / 'metadata' / 'tf.yaml'
    with open(yaml_path) as handle:
        tree = yaml.safe_load(handle)

    frames = {}
    for child_frame_id, entry in tree.items():
        rotation = entry['rotation']
        translation = entry['translation']
        frames[child_frame_id] = {
            'base_frame_id': entry['base_frame_id'],
            'translation': (
                translation['x'], translation['y'], translation['z']),
            'rotation': (
                rotation['x'], rotation['y'], rotation['z'], rotation['w']),
        }
    return frames
