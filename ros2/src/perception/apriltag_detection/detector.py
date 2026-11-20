"""Pure-Python AprilTag detection helpers (no ROS dependency)."""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class TagDetection:
    """One decoded tag."""

    id: int  # noqa: A003
    hamming: int
    margin: float
    center: Tuple[float, float]
    corners: np.ndarray  # 4x2 pixel coordinates
    pose: Optional[dict] = None  # keys t (3x1), R (3x3), error


class TagDetector:
    """Filtering wrapper around the AprilRobotics ``apriltag`` detector."""

    def __init__(self, family='tagStandard41h12', tag_size=0.0378, valid_ids=(),
                 max_hamming=0, min_margin=0.0, **detector_kwargs):
        """Create the detector; an empty ``valid_ids`` accepts every id."""
        try:
            import apriltag
        except ImportError as err:
            raise ImportError(
                "Python module 'apriltag' not found. Run "
                '`colcon build --packages-select apriltag` and then '
                '`source install/setup.bash`.') from err
        self._detector = apriltag.apriltag(family, **detector_kwargs)
        self.tag_size = tag_size
        self.valid_ids = set(valid_ids)
        self.max_hamming = max_hamming
        self.min_margin = min_margin

    def detect(self, gray, camera_k=None) -> List[TagDetection]:
        """
        Detect tags in a uint8 gray image, best (largest margin) first.

        Pass ``camera_k=(fx, fy, cx, cy)`` to also estimate poses.
        """
        found = []
        for d in self._detector.detect(gray):
            if self.valid_ids and d['id'] not in self.valid_ids:
                continue
            if d['hamming'] > self.max_hamming or d['margin'] < self.min_margin:
                continue
            pose = None
            if camera_k is not None:
                pose = self._detector.estimate_tag_pose(d, self.tag_size, *camera_k)
            found.append(TagDetection(
                id=int(d['id']), hamming=int(d['hamming']), margin=float(d['margin']),
                center=(float(d['center'][0]), float(d['center'][1])),
                corners=np.asarray(d['lb-rb-rt-lt'], dtype=float), pose=pose))
        return sorted(found, key=lambda t: t.margin, reverse=True)


class StableTagVote:
    """Confirm a tag id after it is seen in consecutive frames."""

    def __init__(self, confirm_frames: int, lock: bool = False):
        """
        Require ``confirm_frames`` consecutive sightings.

        With ``lock`` the first confirmed id is kept until ``reset()``, so tags
        seen later (e.g. the pit tags during the race) cannot replace it.
        """
        self.confirm_frames = confirm_frames
        self.lock = lock
        self.reset()

    def reset(self):
        """Forget the confirmed id and any streak in progress."""
        self._candidate = None
        self._count = 0
        self._confirmed = None

    def update(self, tag_id: Optional[int]) -> Optional[int]:
        """Feed one frame's best id (or None); return the confirmed id or None."""
        if self.lock and self._confirmed is not None:
            return self._confirmed
        if tag_id is None or tag_id != self._candidate:
            self._candidate = tag_id
            self._count = 0
        if tag_id is not None:
            self._count += 1
            if self._count >= self.confirm_frames:
                self._confirmed = tag_id
        return self._confirmed


__all__: Sequence[str] = ['TagDetection', 'TagDetector', 'StableTagVote']
