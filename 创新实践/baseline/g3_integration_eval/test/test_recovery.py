import pytest
from g3_integration_eval.recovery import RecoveryHealth
from g3_integration_eval.state_machine import State, Event, transition, InvalidTransition, battery_return_event


def test_pause_requires_resume_and_never_bypasses_estop():
    assert transition(State.EXPLORE, Event.PAUSE) is State.PAUSED
    assert transition(State.PAUSED, Event.RESUME) is State.EXPLORE
    assert transition(State.PAUSED, Event.RETURN) is State.RETURN
    with pytest.raises(InvalidTransition): transition(State.PAUSED, Event.EXPLORE)
    with pytest.raises(InvalidTransition): transition(State.ESTOP, Event.RESUME)
    assert battery_return_event(State.PAUSED, .1) is Event.RETURN


def test_all_sources_must_recover_continuously():
    h=RecoveryHealth(['scan','localization'],stale_s=2,stable_s=1,timeout_s=10)
    assert h.missing(0)==['scan','localization']
    h.observe('scan',0)
    assert not h.ready(0)
    h.observe('localization',.5)
    assert not h.ready(.5)
    assert not h.ready(1.4)
    assert h.ready(1.5)
    assert not h.ready(3)
    for k in h.sources:h.observe(k,3)
    assert not h.ready(3)
    assert h.ready(4)


def test_replayed_stamps_and_frozen_clock_do_not_keep_health_alive():
    h=RecoveryHealth(['scan'],stale_s=2,stable_s=1,timeout_s=10)
    h.observe('scan',0,100)
    h.observe('scan',3,100)
    h.observe('scan',3,99)
    assert h.missing(3)==['scan']
    h.observe('scan',3,101)
    assert not h.missing(3)


@pytest.mark.parametrize('values', [dict(stale_s=0),dict(stable_s=float('nan')),dict(timeout_s=1)])
def test_invalid_budgets_rejected(values):
    with pytest.raises(ValueError):RecoveryHealth(['scan'],**values)
