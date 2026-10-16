import importlib.util
from pathlib import Path
import pytest

path=Path(__file__).parents[1]/'scripts/evaluate_fault_recovery.py'
spec=importlib.util.spec_from_file_location('fault_audit',path);audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def episode():
    return dict(start=dict(wall=10.,source='scan'),release=dict(wall=17.),
        states=[(12.,'paused'),(20.,'exploring')],recovery=[dict(status='fault',wall=12.,missing=['scan']),
            dict(status='paused',wall=12.),dict(status='navigation_released',wall=12.3),dict(status='resumed',wall=20.)],
        commands=[(10.,0,.02,0),(11.,0,0,0),(13.,0,0,0),(16.,0,0,0)],
        poses=[(7.,0.,0.),(9.9,.05,0.),(13.,.06,0.),(16.,.06,0.),(20.,.06,0.),(25.,.2,0.)],
        raw_scan=[11.,12.,13.,14.],scan=[],raw_tf=[],tf=[],next_start=26.,mode='short')


def test_complete_short_episode():
    assert audit.audit_episode(**episode())['success']


@pytest.mark.parametrize('field,value,failed',[('scan',[13.],'source_was_cut'),
    ('commands',[(10.,0,.02,0),(14.,0,.02,0),(16.,0,0,0)],'paused_commands_zero'),
    ('poses',[(7.,0.,0.),(9.9,.05,0.),(13.,.06,0.),(16.,.4,0.),(25.,.5,0.)],'paused_body_drift_bounded'),
    ('recovery',[],'paused'),('raw_scan',[],'source_was_cut')])
def test_bad_motion_or_source_cannot_pass(field,value,failed):
    data=episode();data[field]=value;result=audit.audit_episode(**data)
    assert not result['success'] and not result['checks'][failed]


def test_long_fault_requires_latched_estop_after_actual_restore():
    data=episode();data.update(mode='long',release=dict(wall=75.),next_start=90.,
        states=[(12.,'paused'),(72.,'estop')],poses=[(7.,0.,0.),(9.9,.05,0.),(13.,.06,0.),(74.,.06,0.),(84.,.06,0.)],
        commands=[(10.,0,.02,0),(11.,0,0,0),(13.,0,0,0),(74.,0,0,0),(83.,0,0,0)],scan=[76.,77.,78.])
    data['recovery']=data['recovery'][:-1]
    assert audit.audit_episode(**data)['success']
    data['states'].append((78.,'exploring'))
    assert not audit.audit_episode(**data)['success']


def test_return_requires_matching_real_action_uuid_and_body_home():
    events=[dict(status='accepted',attempt_id='return',goal_id='uuid'),
            dict(status='succeeded',attempt_id='return',goal_id='uuid')]
    poses=[(0,0.,0.),(1,.2,0.)]
    assert audit.verified_return(events,{'uuid'},poses)
    assert not audit.verified_return(events,{'different_uuid'},poses)
    assert not audit.verified_return(events,{'uuid'},[(0,0.,0.),(1,1.,0.)])
    events[-1]['attempt_id']='old'
    assert not audit.verified_return(events,{'uuid'},poses)


def test_data_restore_does_not_permit_motion_before_resume():
    data=episode();data['commands'].append((18.,0,.02,0))
    result=audit.audit_episode(**data)
    assert not result['success'] and not result['checks']['paused_commands_zero']


def test_cancel_requires_old_action_terminal_before_resume():
    statuses=[(8.,'old',2),(12.,'old',5),(21.,'new',2)]
    assert audit.released_action(statuses,10.,20.)
    assert not audit.released_action(statuses,10.,11.)
    assert not audit.released_action([],10.,20.)
    assert not audit.released_action([(8.,'old',2),(8.,'other',2),(12.,'old',5)],10.,20.)


def test_goal_accepted_in_same_tick_as_fault_still_requires_real_cancellation():
    # Cross-topic DDS receipt ordering can put accepted just after the stimulus.
    statuses=[(9.,'previous',4),(10.001,'old',2),(12.1,'old',5)]
    assert audit.released_action(statuses,10.,20.,paused=12.)
    assert not audit.released_action(statuses[:-1],10.,20.,paused=12.)
