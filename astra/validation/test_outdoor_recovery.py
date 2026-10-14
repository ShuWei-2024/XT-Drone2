"""A larger outdoor region must still reject collision prisms and disconnected islands."""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from outdoor_recovery import recovery_rectangles
from scan_mesh_guard import triangle_distances


def test_exact_prism_refinement_recovers_false_bbox_exclusions():
    tri=np.array([[[1,1,.2],[3,3,.2],[3,3,2.]]],float)
    class ExactGuard:
        def clearance(self,p):return triangle_distances(p,tri).min()
    old=recovery_rectangles(tri,[[-2,-2],[4,4]],[.2,2.],step=.25)
    new=recovery_rectangles(tri,[[-2,-2],[4,4]],[.2,2.],step=.25,refine_guard=ExactGuard())
    assert new['recovery_certificate']['area_m2']>old['recovery_certificate']['area_m2']
    assert new['recovery_certificate']['refined_cell_count']>0
    for lo,hi in new['recovery_xy_boxes_m']:
        for x in [lo[0],hi[0]]:
            for y in [lo[1],hi[1]]:
                for z in np.linspace(.2,2,11):
                    assert triangle_distances([x,y,z],tri).min()>=.65


def test_vertical_gap_ignores_far_canopy_but_low_obstacle_excludes_cells():
    high=np.array([[[1,-1,5],[2,-1,5],[1,1,5]]],dtype=float)
    bounds=[[-2,-2],[2,2]]
    full=recovery_rectangles(high,bounds,[.2,2.],step=.25)
    assert full['recovery_certificate']['area_m2']==16.
    low=high.copy();low[:,:,2]=1.
    reduced=recovery_rectangles(low,bounds,[.2,2.],step=.25)
    assert reduced['recovery_certificate']['area_m2']<16.
    # Corners and arbitrary interior heights verify the conservative bound
    # against exact triangles, rather than merely repeating bounding-box math.
    for lo,hi in reduced['recovery_xy_boxes_m']:
        for x,y in [lo,hi,(np.array(lo)+hi)/2]:
            for z in [.2,1.,2.]:
                assert triangle_distances([x,y,z],low).min()>=.65


def test_region_only_contains_component_connected_to_spawn():
    wall=np.array([[[1,-4,0],[1,4,0],[1,4,4]],[[1,-4,0],[1,4,4],[1,-4,4]]],float)
    r=recovery_rectangles(wall,[[-2,-2],[3,2]],[.2,2.])
    assert all(hi[0]<1 for lo,hi in r['recovery_xy_boxes_m'])


def test_collision_at_spawn_cannot_be_marked_as_safe():
    t=np.array([[[-1,-1,1],[1,-1,1],[0,1,1]]],float)
    with pytest.raises(ValueError,match='at spawn'):recovery_rectangles(t,[[-2,-2],[2,2]],[.2,2.])


def test_pruned_rows_match_unpruned_rectangle_certificate():
    # Include a huge triangle with a distant centroid and random small faces.
    rng=np.random.default_rng(11)
    tri=rng.uniform([1.,-4.,0.],[5.,4.,2.],(30,3,3))
    tri=np.concatenate([tri,np.array([[[1,-20,1],[1,20,1],[20,0,1]]])])
    result=recovery_rectangles(tri,[[-3,-3],[3,3]],[.2,2.],step=.25)
    xs=ys=np.linspace(-3,3,25)
    lo=tri[:,:,:2].min(1);hi=tri[:,:,:2].max(1)
    from scipy.ndimage import label
    free=np.ones((24,24),bool)
    for i in range(24):
        for j in range(24):
            delta=np.maximum(np.maximum(lo-[xs[i+1],ys[j+1]],[xs[i],ys[j]]-hi),0)
            free[i,j]=np.linalg.norm(delta,axis=1).min()>=.65
    components,_=label(free);spawn=components[11,11]
    for i in range(24):
        for j in range(24):
            centre=np.array([(xs[i]+xs[i+1])/2,(ys[j]+ys[j+1])/2])
            contained=any(np.all(centre>=a) and np.all(centre<=b) for a,b in result['recovery_xy_boxes_m'])
            assert contained==(components[i,j]==spawn)
