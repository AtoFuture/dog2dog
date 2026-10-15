import pytest
from qwen_robotnav.recon_audit import match_targets


def test_one_to_one_rejects_duplicate_person_counts():
    result=match_targets([[0,0],[.1,0]],[[0,0],[2,0]])
    assert sum(v is not None for v in result)==1


def test_assignment_maximizes_people_before_minimizing_error():
    assert match_targets([[.2,0],[0,0]],[[0,0],[.6,0]],.5)==[1,0]
    assert match_targets([[9,9]],[[0,0]])==[None]
    assert match_targets([],[[0,0]])==[]
    with pytest.raises(ValueError):match_targets([],[],float('nan'))
