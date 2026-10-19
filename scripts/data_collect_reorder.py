"""Bounded raw-packet ordering shared by CDC and HID collectors.

Only unresolved holes delay output. Sequence positions more than WINDOW behind
high-water are no longer repairable; unseen late data is retained as an unknown
boundary, never silently discarded or interpreted as a 16-bit-sized loss.
"""

import heapq


class SampleReorder:
    WINDOW = 200
    MAX_FORWARD_GAP = 500

    def __init__(self, on_sample):
        self.on_sample = on_sample
        self.pending = {}
        self._positions = []
        self._recent = [None] * (self.WINDOW + 1)
        self._high = None
        self._cursor = None
        self._last = None
        self.gap_count = 0
        self.retransmit_count = 0
        self.unknown_boundaries = 0

    def _emit(self, position):
        sample, discontinuity = self.pending.pop(position)
        if self._last is not None and not discontinuity:
            gap = position - self._last - 1
            if gap > self.MAX_FORWARD_GAP:
                discontinuity = True
            else:
                self.gap_count += gap
        if discontinuity:
            self.unknown_boundaries += 1
        self.on_sample(sample, discontinuity)
        self._last = position
        self._cursor = position + 1

    def _drain(self, final=False):
        while self._positions:
            position = self._positions[0]
            if not final and position != self._cursor:
                if self._high - self._cursor <= self.WINDOW:
                    break
                self._cursor += 1
                continue
            heapq.heappop(self._positions)
            self._emit(position)

    def _start(self, sample, payload, discontinuity):
        self.finish()
        self._recent = [None] * (self.WINDOW + 1)
        position = sample["seq"]
        self._high = position
        self._cursor = position
        self._last = None
        self._insert(position, sample, payload, discontinuity)

    def _insert(self, position, sample, payload, discontinuity=False):
        self._recent[position % len(self._recent)] = (position, payload)
        self.pending[position] = (sample, discontinuity)
        heapq.heappush(self._positions, position)
        self._drain()

    def push(self, sample, payload, discontinuity=False):
        """Accept a parsed sample and immutable ESB payload (no USB footer).

        Return False only for an exact duplicate. An explicit boundary must be
        associated with this sample; metadata arrival alone is not an epoch.
        """
        if self._high is None or discontinuity:
            self._start(sample, payload, discontinuity)
            return True
        delta = ((sample["seq"] - self._high + 32768) & 0xFFFF) - 32768
        position = self._high + delta
        if position < self._high - self.WINDOW or delta > self.MAX_FORWARD_GAP + 1:
            self._start(sample, payload, True)
            return True
        recent = self._recent[position % len(self._recent)]
        if recent is not None and recent[0] == position:
            if recent[1] == payload:
                self.retransmit_count += 1
                return False
            # Same sequence, different full payload: retain both epochs.
            self._start(sample, payload, True)
            return True
        if position < self._cursor:
            self._start(sample, payload, True)
            return True
        if delta < 0:
            self.retransmit_count += 1
        self._high = max(self._high, position)
        self._insert(position, sample, payload)
        return True

    def finish(self):
        """Commit the remaining ordered tail only at a true stop/boundary."""
        self._drain(final=True)
