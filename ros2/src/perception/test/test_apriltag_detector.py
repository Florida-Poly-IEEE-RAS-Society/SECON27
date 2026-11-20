"""Tests for the AprilTag detector helpers using the simulation tag textures."""

from pathlib import Path

from apriltag_detection.detector import StableTagVote, TagDetector
import cv2
import pytest

MODELS = Path(__file__).resolve().parents[4] / 'simulation' / 'models'


def load_tag(tag_id):
    paths = sorted((MODELS / f'apriltag_41h12_{tag_id}' / 'materials' / 'textures').glob('*.png'))
    img = cv2.imread(str(paths[0]), cv2.IMREAD_GRAYSCALE)
    return cv2.copyMakeBorder(img, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)


@pytest.mark.parametrize('tag_id', [1, 2, 3])
def test_detects_sim_tag(tag_id):
    tags = TagDetector(valid_ids=[1, 2, 3]).detect(load_tag(tag_id))
    assert [t.id for t in tags] == [tag_id]
    assert tags[0].corners.shape == (4, 2)
    assert tags[0].pose is None


def test_pose_with_intrinsics():
    tags = TagDetector().detect(load_tag(1), camera_k=(500.0, 500.0, 320.0, 240.0))
    assert tags[0].pose is not None and tags[0].pose['t'] is not None


def test_valid_ids_filter():
    assert TagDetector(valid_ids=[2, 3]).detect(load_tag(1)) == []


def test_stable_vote():
    vote = StableTagVote(3)
    assert [vote.update(1), vote.update(1)] == [None, None]
    assert vote.update(1) == 1
    assert vote.update(None) == 1  # stays confirmed
    assert [vote.update(2), vote.update(2)] == [1, 1]
    assert vote.update(2) == 2  # switch needs N frames
    assert [vote.update(3), vote.update(None), vote.update(3), vote.update(3)] == [2] * 4
    assert vote.update(3) == 3


def test_stable_vote_lock():
    vote = StableTagVote(2, lock=True)
    assert [vote.update(1), vote.update(1)] == [None, 1]
    assert [vote.update(3), vote.update(3), vote.update(3)] == [1, 1, 1]  # pit tags ignored
    vote.reset()
    assert [vote.update(None), vote.update(3), vote.update(3)] == [None, None, 3]
