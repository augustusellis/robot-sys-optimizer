"""v12 item 16: core.transforms on scipy Rotation, the gimbal-lock convention kept.

Done when the matrix -> rpy / quaternion conversions give the v11 values (1e-12), gimbal-lock
matrices still put all rotation into yaw (roll 0), project poses round-trip, and rpy_to_matrix
(kept closed form for speed) equals scipy's "xyz".
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from core import transforms as tf
from core.schema import Pose


def _v11_rpy(R):
    sp = -R[2, 0]
    if abs(sp) > 1 - 1e-12:
        return 0.0, float(np.copysign(np.pi / 2, sp)), float(np.arctan2(-R[0, 1], R[1, 1]))
    return float(np.arctan2(R[2, 1], R[2, 2])), float(np.arcsin(sp)), float(np.arctan2(R[1, 0], R[0, 0]))


RNG = np.random.default_rng(12)
RPYS = [RNG.uniform(-np.pi, np.pi, 3) for _ in range(200)]


def test_rpy_to_matrix_is_scipy_xyz():
    for rpy in RPYS:
        assert np.allclose(tf.rpy_to_matrix(rpy), Rotation.from_euler("xyz", rpy).as_matrix(), atol=1e-14)


def test_matrix_to_rpy_matches_v11_and_round_trips():
    for rpy in RPYS:
        R = tf.rpy_to_matrix(rpy)
        assert np.allclose(tf.matrix_to_rpy(R), _v11_rpy(R), atol=1e-12)
        assert np.allclose(tf.rpy_to_matrix(tf.matrix_to_rpy(R)), R, atol=1e-12)


@pytest.mark.parametrize("pitch", [np.pi / 2, -np.pi / 2])
def test_gimbal_lock_puts_rotation_into_yaw(pitch):
    R = tf.rpy_to_matrix((0.4, pitch, 0.9))
    roll, p, yaw = tf.matrix_to_rpy(R)
    assert roll == 0.0 and p == pytest.approx(pitch)
    assert np.allclose(tf.rpy_to_matrix((roll, p, yaw)), R, atol=1e-9)


def test_quaternions_match_v11():
    for rpy in RPYS:
        R = tf.rpy_to_matrix(rpy)
        q = tf.matrix_to_wxyz(R)
        assert q[0] >= 0 and np.isclose(np.linalg.norm(q), 1.0)
        assert np.allclose(tf.wxyz_to_matrix(q), R, atol=1e-12)
        assert np.allclose(tf.wxyz_to_matrix(2.5 * q), R, atol=1e-12)  # not normalised: still R


def test_pose_round_trip():
    p = Pose(xyz=(0.1, -0.2, 0.3), rpy=(0.5, -0.4, 2.0))
    back = tf.matrix_to_pose(tf.pose_to_matrix(p))
    assert np.allclose(back.xyz, p.xyz) and np.allclose(back.rpy, p.rpy, atol=1e-12)
