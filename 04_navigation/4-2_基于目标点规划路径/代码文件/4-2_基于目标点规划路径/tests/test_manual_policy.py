import copy
import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_2.manual_policy import destination_check
from lite3_lab_5_2.manual_view import configure_manual_view


def check(data,goal=(2.,2.,0.),origin=(0.,0.,0.)):
    return destination_check(data,80,80,.05,origin,goal,.779,.480,.10)


def test_free_goal_and_nonlethal_inflation_are_not_confused_with_obstacles():
    cells=[0]*6400
    cells[40*80+40]=70
    result=check(cells)
    assert result['passed'] and result['center_cost']==70


@pytest.mark.parametrize('cost',[-1,99,100])
def test_unknown_inscribed_and_lethal_goal_centers_are_rejected(cost):
    cells=[0]*6400
    cells[40*80+40]=cost
    assert not check(cells)['passed']


def test_free_center_cannot_hide_obstacle_or_unknown_under_full_goal_footprint():
    cells=[0]*6400
    cells[44*80+46]=100
    assert not check(cells)['passed']
    cells[44*80+46]=-1
    assert not check(cells)['passed']


def test_goal_near_map_edge_is_rejected_and_rotated_map_origin_is_supported():
    cells=[0]*6400
    assert not check(cells,(.1,.1,0.))['passed']
    assert check(cells,(-2.,2.,math.pi/2),(0.,0.,math.pi/2))['passed']


def test_manual_view_routes_clicks_through_client_and_preserves_original_profile():
    original={'Visualization Manager':{'Global Options':{'Fixed Frame':'odom'},'Displays':[],
        'Tools':[{'Class':'nav2_rviz_plugins/GoalTool'}, {'Class':'rviz_default_plugins/Interact'}]}}
    before=copy.deepcopy(original)
    result=configure_manual_view(original)
    assert original==before
    tools=result['Visualization Manager']['Tools']
    assert all(t['Class']!='nav2_rviz_plugins/GoalTool' for t in tools)
    goal=next(t for t in tools if t['Class']=='rviz_default_plugins/SetGoal')
    assert goal['Topic']['Value']=='/lab5/manual_goal'
    assert result['Visualization Manager']['Displays'][0]['Topic']['Value']=='/lab5/selected_goal'
