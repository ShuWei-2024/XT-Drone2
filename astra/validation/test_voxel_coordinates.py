"""Numerical regressions for mixed-unit neighbours and negative grid boundaries."""
from pathlib import Path
import subprocess


def test_coordinate_boundaries_and_neighbour_direction(tmp_path):
    include=Path(__file__).resolve().parents[1]/'packages/astra_fast_livo2_core/include'
    source=tmp_path/'coordinates.cpp'
    source.write_text('''
#include "voxel_coordinates.h"
#include <cassert>
int main() {
  assert(voxel_coordinate(-.5,.5)==-1);
  assert(voxel_coordinate(-.500001,.5)==-2);
  assert(voxel_coordinate(-.499999,.5)==-1);
  assert(voxel_coordinate(0.,.5)==0);
  assert(voxel_coordinate(.5,.5)==1);
  // At x=1.1 m the 0.5 m cell's left neighbour is appropriate;
  // comparing its dimensionless 2.2 key to a metre boundary picks the right.
  assert(neighbouring_voxel_offset(1.1,1.25,.125)==-1);
  assert(neighbouring_voxel_offset(-.6,-.75,.125)==1);
  assert(neighbouring_voxel_offset(1.25,1.25,.125)==0);
  for (double scale : {.25,.5,1.,2.}) {
    assert(neighbouring_voxel_offset(2.2*scale,2.5*scale,.25*scale)==-1);
    assert(neighbouring_voxel_offset(-1.2*scale,-1.5*scale,.25*scale)==1);
  }
}
''')
    binary=tmp_path/'coordinates'
    subprocess.run(['g++','-std=c++17','-include','initializer_list','-I',str(include),
                    str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
