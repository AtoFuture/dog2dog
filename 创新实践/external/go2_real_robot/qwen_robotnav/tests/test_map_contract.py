import copy
from types import SimpleNamespace as NS
import pytest
from qwen_robotnav.map_contract import planning_map_preserves


def pair():
    q=NS(x=0.,y=0.,z=0.,w=1.)
    raw=NS(header=NS(frame_id='map',stamp=(5,0)),info=NS(width=2,height=1,resolution=.1,
        origin=NS(position=NS(x=1.,y=2.,z=0.),orientation=q)),data=[0,100])
    planned=copy.deepcopy(raw);planned.info.width=4;planned.info.height=3
    planned.info.origin.position.x=.9;planned.info.origin.position.y=1.9
    planned.data=[-1,-1,-1,-1,-1,0,100,-1,-1,-1,-1,-1]
    return raw,planned


def test_only_unknown_border_is_valid():
    raw,planned=pair();assert planning_map_preserves(raw,planned,.1)


@pytest.mark.parametrize('change',[lambda p:p.data.__setitem__(0,0),lambda p:p.data.__setitem__(6,0),
    lambda p:setattr(p.info.origin.position,'x',1.),lambda p:setattr(p.header,'frame_id','odom'),
    lambda p:setattr(p.header,'stamp',(6,0)),lambda p:setattr(p.info,'resolution',.2)])
def test_invented_free_space_changed_observations_or_frame_shift_are_rejected(change):
    raw,planned=pair();change(planned)
    assert not planning_map_preserves(raw,planned,.1)
