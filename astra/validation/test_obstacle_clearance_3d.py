"""Ceilings and obstacle tops must use distance in all three dimensions."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from obstacle_world_geometry_20261010 import clearances, extract


def test_ceiling_is_safe_below_but_detects_approach():
    roof=dict(kind='box',center=[0,0,4.15],size=[12,13,.3])
    assert clearances([[0,0,2.34]],[roof])[0] == pytest.approx(1.66)
    assert clearances([[0,0,3.6]],[roof])[0] == pytest.approx(.4)
    assert clearances([[0,0,4.1]],[roof])[0] == 0
    assert clearances([[0,0]],[roof])[0] == 0


def test_box_corner_and_cylinder_cap():
    box=dict(kind='box',center=[0,0,1],size=[2,2,2])
    cylinder=dict(kind='cylinder',center=[0,0,1],radius=1,height=2)
    assert clearances([[2,0,3]],[box])[0] == pytest.approx(np.sqrt(2))
    assert clearances([[2,0,3]],[cylinder])[0] == pytest.approx(np.sqrt(2))
    assert clearances([[.5,0,1]],[cylinder])[0] == 0


def test_hall_has_four_walls_and_roof_and_raw_blocked_line():
    world=Path(__file__).resolve().parents[1]/'packages/astra_gz_sensors/worlds/astra_px4_indoor_hall.sdf'
    obstacles=extract(world)
    assert {'indoor_hall_west_wall','indoor_hall_ceiling'}.issubset({o['name'] for o in obstacles})
    line=np.column_stack([np.linspace(0,5,1001),np.linspace(0,2.5,1001),np.full(1001,1.94)])
    assert clearances(line,obstacles).min() == 0
    assert clearances([[0,0,.24]],obstacles)[0] > 3


@pytest.mark.parametrize('points', [[[float('nan'),0,0]], [[0]], [[0,0,0,0]]])
def test_invalid_positions_rejected(points):
    with pytest.raises(ValueError):clearances(points,[])
