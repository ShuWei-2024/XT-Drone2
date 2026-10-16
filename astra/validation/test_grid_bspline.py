"""Verify fallback boundary states, obstacle rounding and float arc sampling."""
from pathlib import Path
import subprocess


def test_sensor_grid_spline_boundary_and_detour(tmp_path):
    package=Path(__file__).resolve().parents[1]/'packages/ego_planner_jazzy/bspline_opt'
    source=tmp_path/'grid.cpp'
    source.write_text(r'''
#include <bspline_opt/grid_bspline.h>
#include <bspline_opt/retime_bspline.h>
#include <cassert>
using namespace ego_planner;
int main() {
  const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> straight;
  for(int i=0;i<41;++i) straight.emplace_back(1.3+i*.2,-1.7,1.);
  auto line=gridPathSpline(straight,zero,zero,zero,.2,1.);
  assert(line.getControlPoint().cols()>=44);
  for(int i=3;i<line.getControlPoint().cols()-3;++i)
    assert((line.getControlPoint().col(i)-line.getControlPoint().col(i-1)).norm()<=.20000001);
  std::vector<Eigen::Vector3d> route{{0.,0.,1.},{1.5,-3.,1.},{5.5,-3.,1.},{7.,0.,1.}};
  Eigen::Vector3d velocity(.03,-.04,0.),acceleration(.01,-.005,0.),terminal(.02,0.,0.);
  auto curve=gridPathSpline(route,velocity,acceleration,terminal,.2,1.);
  assert((curve.evaluateDeBoorT(0.)-route.front()).norm()<1e-12);
  assert((curve.getDerivative().evaluateDeBoorT(0.)-velocity).norm()<1e-12);
  assert((curve.getDerivative().getDerivative().evaluateDeBoorT(0.)-acceleration).norm()<1e-12);
  assert((curve.evaluateDeBoorT(curve.getTimeSum())-route.back()).norm()<1e-12);
  assert((curve.getDerivative().evaluateDeBoorT(curve.getTimeSum())-terminal).norm()<1e-12);
  const double scale=retimeCollisionFreeSpline(curve,.2,.25);
  assert((curve.getDerivative().evaluateDeBoorT(0.)-velocity/scale).norm()<1e-12);
  for(int i=0;i<=2000;++i) {
    const double t=curve.getTimeSum()*i/2000.;
    auto p=curve.evaluateDeBoorT(t);
    // Independent distance to the physical test box, after spline rounding.
    const double dx=std::max({3.1-p.x(),0.,p.x()-3.9});
    const double dy=std::max({-1.5-p.y(),0.,p.y()-1.5});
    assert(std::hypot(dx,dy)>.55);
    assert(std::abs(p.z()-1.)<1e-12);
    assert(curve.getDerivative().evaluateDeBoorT(t).norm()<=.2);
    assert(curve.getDerivative().getDerivative().evaluateDeBoorT(t).norm()<=.25);
  }
  auto stop=gridPathSpline(route,zero,zero,zero,.2,1.);
  assert(stop.getDerivative().evaluateDeBoorT(stop.getTimeSum()).norm()<1e-12);
}
''')
    binary=tmp_path/'grid'
    subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/eigen3',
                    '-I',str(package/'include'),str(source),
                    str(package/'src/uniform_bspline.cpp'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
