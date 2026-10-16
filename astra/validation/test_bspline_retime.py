"""Exercise production time allocation on a curved obstacle detour."""
from pathlib import Path
import subprocess


def test_retime_keeps_geometry_and_bounds_derivatives(tmp_path):
    package = Path(__file__).resolve().parents[1]/'packages/ego_planner_jazzy/bspline_opt'
    source = tmp_path/'retime.cpp'
    source.write_text(r'''
#include <bspline_opt/retime_bspline.h>
#include <cassert>
using namespace ego_planner;
int main() {
  Eigen::MatrixXd controls(3,10);
  controls << 0,0,0,.2,.4,.6,.8,1,1,1,
              0,0,0,.3,.7,.7,.3,0,0,0,
              1,1,1,1,1,1,1,1,1,1;
  UniformBspline original(controls,3,1.0), slowed=original;
  const double scale=retimeCollisionFreeSpline(slowed,.20,.25);
  assert(scale>1.01);
  assert((slowed.getControlPoint()-controls).norm()==0.);
  assert(std::abs(slowed.getTimeSum()/original.getTimeSum()-scale)<1e-12);
  for(int i=0;i<=1000;++i) {
    const double t=original.getTimeSum()*i/1000.;
    assert((original.evaluateDeBoorT(t)-slowed.evaluateDeBoorT(t*scale)).norm()<1e-12);
    assert(slowed.getDerivative().evaluateDeBoorT(t*scale).norm()<=.20);
    assert(slowed.getDerivative().getDerivative().evaluateDeBoorT(t*scale).norm()<=.25);
  }
  assert(slowed.getDerivative().evaluateDeBoorT(slowed.getTimeSum()).norm()<1e-12);
  for(double invalid : {0.,-1.,std::numeric_limits<double>::infinity()}) {
    bool rejected=false;
    try { retimeCollisionFreeSpline(slowed,invalid,.25); }
    catch(const std::invalid_argument &) { rejected=true; }
    assert(rejected);
  }
  bool rejected=false;
  try { retimeCollisionFreeSpline(original,.0001,.0001); }
  catch(const std::invalid_argument &) { rejected=true; }
  assert(rejected);
  // Repeated allocation must not multiply down the handoff velocity.
  UniformBspline continuous(controls,3,1.);
  const Eigen::Vector3d start=continuous.evaluateDeBoorT(0.);
  const Eigen::Vector3d handoff(.12,-.02,0.);
  for(int replans=0;replans<50;++replans) {
    retimeStartContinuousSpline(continuous,start,handoff,.20,.25);
    assert((continuous.evaluateDeBoorT(0.)-start).norm()<1e-12);
    assert((continuous.getDerivative().evaluateDeBoorT(0.)-handoff).norm()<1e-12);
    assert(continuous.getDerivative().getControlPoint().colwise().norm().maxCoeff()<=.20+1e-9);
    assert(continuous.getDerivative().getDerivative().getControlPoint().colwise().norm().maxCoeff()<=.25+1e-9);
    assert((continuous.evaluateDeBoorT(continuous.getTimeSum())-original.evaluateDeBoorT(original.getTimeSum())).norm()<1e-12);
  }
  rejected=false;
  try { retimeStartContinuousSpline(continuous,start,Eigen::Vector3d(.3,0.,0.),.2,.25); }
  catch(const std::invalid_argument &) { rejected=true; }
  assert(rejected);
}
''')
    binary = tmp_path/'retime'
    subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/eigen3',
                    '-I',str(package/'include'),str(source),
                    str(package/'src/uniform_bspline.cpp'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
