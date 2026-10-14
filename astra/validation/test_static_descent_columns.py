"""Observed furniture can block overflight; floor inflation must not fill a room."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'packages/astra_fast_livo2/scripts'))
from local_mapping_guard import inflate_cells, static_descent_obstacles


def test_floor_is_not_extruded_even_when_inflation_reaches_above_threshold():
    floor=np.array([[0,0,-2]])
    offsets=np.array([[0,0,0],[0,0,4]])
    assert np.array_equal(static_descent_obstacles(floor,offsets,.2,.15,1.5),inflate_cells(floor,offsets))


def test_observed_furniture_column_blocks_both_underflight_and_overflight():
    cells=np.array([[0,0,-2],[10,0,3]])
    offsets=np.array([[0,0,0],[1,0,0],[-1,0,0]])
    before=cells.copy()
    result=set(map(tuple,static_descent_obstacles(cells,offsets,.2,.15,1.5)))
    for x in [9,10,11]:
        for z in range(8):assert (x,0,z) in result
    assert (0,0,7) not in result
    assert (0,0,-2) in result
    assert np.array_equal(cells,before)


def test_overhead_ceiling_is_not_projected_down_to_the_floor():
    ceiling=np.array([[0,0,12]])
    offsets=np.array([[0,0,0],[0,0,-1]])
    result=static_descent_obstacles(ceiling,offsets,.2,.15,1.5)
    assert np.array_equal(result,inflate_cells(ceiling,offsets))


def test_wider_horizontal_columns_only_expand_observed_objects():
    cells=np.array([[0,0,-2],[10,0,3],[20,0,12]])
    offsets=np.array([[0,0,0],[1,0,0],[-1,0,0]])
    wider=np.array([[x,0,0] for x in range(-3,4)])
    result=set(map(tuple,static_descent_obstacles(cells,offsets,.2,.15,1.5,wider)))
    for x in range(7,14):
        for z in range(8):assert (x,0,z) in result
    # Floor and ceiling keep their original 3D inflation and height.
    for x in [-1,0,1]:assert (x,0,-2) in result
    assert (-3,0,-2) not in result
    assert (0,0,7) not in result
    assert (10,0,8) not in result
    assert (17,0,12) not in result
    assert (20,0,0) not in result


def test_observed_branches_above_flight_ceiling_can_block_descent():
    cells=np.array([[0,0,-2],[10,0,10],[20,0,20]])
    offsets=np.array([[0,0,0]])
    result=set(map(tuple,static_descent_obstacles(cells,offsets,.2,.15,1.5,
                                                maximum_obstacle_z=3.)))
    assert (10,0,0) in result and (10,0,7) in result
    assert (0,0,7) not in result and (20,0,0) not in result
    assert (20,0,20) in result
