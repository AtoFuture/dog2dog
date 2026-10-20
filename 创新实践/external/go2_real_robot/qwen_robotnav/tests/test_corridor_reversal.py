import math
from dataclasses import replace
from pathlib import Path
import pytest
from qwen_robotnav.corridor import Corridor, CorridorGuidance
from qwen_robotnav.runtime_profile import load_profile, validate_profile


def guide(offset=.12, angle=0.):
    settings=load_profile(Path(__file__).parents[1]/'config/sim_runtime.yaml')['corridor']
    settings['reversal_turn_enabled']=True
    result=CorridorGuidance(settings)
    result.geometry=Corridor(1.4,angle,offset,3.,80,
        positive_wall_start_m=-3.,negative_wall_start_m=-3.)
    return result


@pytest.mark.parametrize('offset,sign',[(.12,-1.),(-.12,1.)])
def test_reversal_chooses_direction_away_from_calibrated_slip(offset,sign):
    g=guide(offset)
    assert g.guide(0.,.4,(-2.,0.))==(0.,sign*.4,'reversal_turning')
    assert g.guide(.32,.8,(-2.,0.))==(0.,sign*.4,'reversal_turning')


def test_choice_survives_wrap_and_zero_request_then_releases_near_alignment():
    g=guide()
    g.guide(0.,.4,(-2.,0.))
    assert g.guide(0.,0.,(-2.,0.))==(0.,0.,'stationary')
    # Clockwise body turn: walls rotate counterclockwise in the base frame.
    for angle in (.5,1.5,2.5):
        g.geometry=replace(g.geometry,heading_rad=angle)
        target=(-2*math.cos(angle),-2*math.sin(angle))
        assert g.guide(.32,.4,target)==(0.,-.4,'reversal_turning')
    g.geometry=replace(g.geometry,heading_rad=math.pi-.1)
    assert g.guide(0.,.2,(2.,0.))==(0.,.2,'turning')
    assert g.snapshot()['reversal_turn_sign'] is None


def test_axis_representation_does_not_change_selected_physical_direction():
    a=guide();b=guide(-.12,math.pi)
    assert a.guide(0.,.4,(-2.,0.))==b.guide(0.,.4,(-2.,0.))


@pytest.mark.parametrize('target', [None,(0.,0.),(.2,0.),(math.nan,0.),(2.,0.)])
def test_no_override_without_a_distant_reverse_path(target):
    assert guide().guide(0.,.3,target)==(0.,.3,'turning')


def test_no_self_start_no_override_for_obstacle_and_explicit_opt_in():
    g=guide()
    assert g.guide(0.,0.,(-2.,0.))==(0.,0.,'stationary')
    assert g.snapshot()['reversal_turn_sign'] is None
    g.geometry=replace(g.geometry,backward_obstacle_m=.8)
    assert g.guide(.32,.6,(-2.,0.))==(.32,.6,'planner_obstacle_avoidance')
    g=guide();g.settings.pop('reversal_turn_enabled')
    assert g.guide(0.,.3,(-2.,0.))==(0.,.3,'turning')
    g=guide();g.guide(0.,.4,(-2.,0.));g.reset_alignment()
    assert g.snapshot()['reversal_turn_sign'] is None


@pytest.mark.parametrize('key',['reversal_turn_enabled','turn_settle_enabled'])
def test_experimental_options_require_booleans(key):
    profile=load_profile(Path(__file__).parents[1]/'config/sim_runtime.yaml')
    assert key not in profile['corridor']
    profile['corridor'][key]=True;validate_profile(profile)
    profile['corridor'][key]=1
    with pytest.raises(ValueError):validate_profile(profile)
