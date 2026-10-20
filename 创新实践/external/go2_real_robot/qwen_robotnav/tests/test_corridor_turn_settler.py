"""Walk/turn transition timing, including changing requests and lost geometry."""
import pytest
from qwen_robotnav.corridor import CorridorTurnSettler


def test_turn_waits_for_both_ramps_and_then_a_complete_rest_boundary():
    gate = CorridorTurnSettler()
    assert gate.filter(.32, .2, .024, .3, 0., True) == (.2, False)
    assert gate.filter(0., .4, .024, .3, 1., True) == (0., True)
    assert gate.filter(0., .4, 0., .1, 2., True) == (0., True)
    assert gate.filter(0., .4, 0., 0., 3., True) == (0., True)
    assert gate.filter(0., -.4, 0., 0., 3.79, True) == (0., True)
    assert gate.filter(0., -.4, 0., 0., 3.81, True) == (-.4, False)
    # No repeated waiting during one continuous turn.
    assert gate.filter(0., -.4, 0., -.5, 4., True) == (-.4, False)


def test_stationary_start_and_disabled_legacy_guidance_are_unchanged():
    gate = CorridorTurnSettler()
    assert gate.filter(0., .4, 0., 0., 0., True) == (.4, False)
    assert gate.filter(.32, .2, .024, .3, 1., False) == (.2, False)
    assert gate.filter(0., .4, .024, .3, 2., False) == (.4, False)


def test_pending_stop_survives_lost_wall_confirmation_and_zero_requests():
    gate = CorridorTurnSettler()
    gate.filter(.32, 0., .024, 0., 0., True)
    assert gate.filter(0., 0., .01, 0., 1., False) == (0., True)
    assert gate.filter(0., 0., 0., 0., 2., False) == (0., True)
    assert gate.filter(0., .4, 0., 0., 2.5, False) == (0., True)
    assert gate.filter(0., .4, 0., 0., 2.81, False) == (.4, False)
    assert gate.filter(0., 0., 0., .3, 3., True) == (0., False)


def test_paused_or_reset_simulation_clock_does_not_expire_wait():
    gate = CorridorTurnSettler()
    gate.filter(.32, 0., .024, 0., 9., True)
    gate.filter(0., .4, 0., 0., 10., True)
    for _ in range(100):
        assert gate.filter(0., .4, 0., 0., 10., True) == (0., True)
    assert gate.filter(0., .4, 0., 0., 1., True) == (0., True)
    assert gate.filter(0., .4, 0., 0., 1.79, True) == (0., True)
    assert gate.filter(0., .4, 0., 0., 1.81, True) == (.4, False)


@pytest.mark.parametrize('interruption', ['walk', 'ramp'])
def test_interrupted_wait_requires_a_new_full_settle_interval(interruption):
    gate = CorridorTurnSettler()
    gate.filter(.32, 0., .024, 0., 0., True)
    gate.filter(0., .4, 0., 0., 1., True)
    if interruption == 'walk':
        assert gate.filter(.32, 0., 0., 0., 1.5, True) == (0., False)
    else:
        assert gate.filter(0., .4, 0., .01, 1.5, True) == (0., True)
    assert gate.filter(0., .4, 0., 0., 2., True) == (0., True)
    assert gate.filter(0., .4, 0., 0., 2.79, True) == (0., True)
    assert gate.filter(0., .4, 0., 0., 2.81, True) == (.4, False)
