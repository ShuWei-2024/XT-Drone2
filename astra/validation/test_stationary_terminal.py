"""Exercise the production boundary helper and cubic endpoint derivatives."""
from pathlib import Path
import subprocess


def test_stationary_cubic_endpoint(tmp_path):
    include = Path(__file__).resolve().parents[1]/'packages/ego_planner_jazzy/bspline_opt/include'
    source = tmp_path/'terminal.cpp'
    source.write_text('''
#include "bspline_opt/stationary_terminal.h"
#include <cassert>
int main() {
  Eigen::MatrixXd controls = Eigen::MatrixXd::Random(3,10);
  auto head = controls.leftCols(3).eval();
  Eigen::Vector3d target(-.3,.15,1.7);
  ego_planner::pinStationaryTerminal(controls,target);
  assert((controls.leftCols(3)-head).norm()==0.);
  auto a=controls.col(7),b=controls.col(8),c=controls.col(9);
  assert(((a+4*b+c)/6-target).norm()<1e-12);
  for(double dt : {.1,1.,10.}) {
    assert(((c-a)/(2*dt)).norm()<1e-12);
    assert(((a-2*b+c)/(dt*dt)).norm()<1e-12);
  }
}
''')
    binary = tmp_path/'terminal'
    subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3','-I',str(include),str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
