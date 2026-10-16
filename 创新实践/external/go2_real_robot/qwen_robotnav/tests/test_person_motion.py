import math
import pytest
from qwen_robotnav.person_motion import positions,SCENARIOS,DURATION_S


@pytest.mark.parametrize('scenario',SCENARIOS)
def test_scripted_people_move_continuously_return_and_keep_physical_separation(scenario):
    previous=positions(scenario,0)
    assert positions(scenario,DURATION_S)==pytest.approx(previous)
    displacement=0.
    for tick in range(1,281):
        current=positions(scenario,tick*.1)
        assert math.dist(*current)>=.59
        assert all(math.dist(a,b)<=.03 for a,b in zip(previous,current))
        displacement=max(displacement,math.dist(current[0],positions(scenario,0)[0]))
        previous=current
    assert displacement>=.5
