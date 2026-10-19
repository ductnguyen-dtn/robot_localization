""" Unit tests for the parts of the particle filter that don't need a
    running ROS node: Particle's pose conversion, and the resampling
    helper that resample_particles() relies on.

    Run with: python3 -m pytest test/test_pf_logic.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'robot_localization'))

from pf import Particle
from helper_functions import draw_random_sample


def test_as_pose_round_trip():
    """ A particle's as_pose() should produce a quaternion that, converted
        back to yaw, gives the same angle the particle was built with. """
    for theta in [0.0, 0.5, -0.5, math.pi / 2, -math.pi / 2, 3.0, -3.0]:
        p = Particle(x=1.0, y=-2.0, theta=theta)
        pose = p.as_pose()
        assert pose.position.x == 1.0
        assert pose.position.y == -2.0

        # convert the quaternion back to yaw by hand, the same way
        # helper_functions.euler_from_quaternion does
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        assert math.isclose(yaw, theta, abs_tol=1e-9)


def test_draw_random_sample_respects_weights():
    """ resample_particles() leans entirely on draw_random_sample to turn
        particle weights into the next generation of particles. With a
        skewed weight distribution and enough draws, the empirical
        fraction of each choice picked should land close to its weight. """
    choices = ['a', 'b', 'c']
    weights = [0.7, 0.2, 0.1]
    n = 20000

    sample = draw_random_sample(choices, weights, n)
    counts = {c: sample.count(c) for c in choices}

    assert len(sample) == n
    for choice, weight in zip(choices, weights):
        empirical_fraction = counts[choice] / n
        assert math.isclose(empirical_fraction, weight, abs_tol=0.02)


def test_draw_random_sample_never_picks_a_zero_weight_choice():
    """ A particle with zero weight should never survive resampling,
        no matter how many times we draw. """
    choices = ['keep', 'drop']
    weights = [1.0, 0.0]
    sample = draw_random_sample(choices, weights, 1000)
    assert 'drop' not in sample
    assert sample.count('keep') == 1000
