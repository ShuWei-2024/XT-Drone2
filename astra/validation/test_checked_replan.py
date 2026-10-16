"""Keep a safe curved handoff; reject new obstacles, expiry and exhausted work."""
from pathlib import Path
import subprocess


def test_checked_old_curve_at_failed_tangent_replan(tmp_path):
    package = Path(__file__).resolve().parents[1] / 'packages/ego_planner_jazzy/bspline_opt'
    source = tmp_path / 'checked.cpp'
    source.write_text(r'''
#include <bspline_opt/checked_replan.h>
#include <cassert>
using namespace ego_planner;
int main() {
  // First six controls of trajectory 23 in outdoor_20261016_153602.
  Eigen::MatrixXd p(3,6);
  p << 1.7943401109505361,1.8447474649380864,1.870833926433446,
       1.9182282377525561,1.8260148606903766,1.733801483628197,
       1.6569979028328765,1.7226883407372822,1.7775243170123094,
       1.843045031838657,1.8817321497306414,1.9204192676226255,
       1.37737698,1.37737698,1.37737698,1.37737698,1.37737698,1.37737698;
  UniformBspline old(p,3,.7985119030269885);
  const auto original = old.getControlPoint();
  const auto deadline = [] { return std::chrono::steady_clock::now()+std::chrono::seconds(1); };
  const double boundary=1.9002958768986733;
  const auto safe = [&](const Eigen::Vector3d &v) {return v.x()<boundary;};
  assert(!safe(Eigen::Vector3d(1.901,1.827,1.377))); // actual rejected replacement
  assert(checkedReplanSuffix(old,1.088,.5,.01,safe,deadline()));
  // A new occupied area on the old curve must prevent retention too.
  assert(!checkedReplanSuffix(old,1.088,.5,.01,[&](const Eigen::Vector3d &v) {
    return safe(v) && v.y()<1.84;
  },deadline()));
  assert(!checkedReplanSuffix(old,old.getTimeSum()-.1,.5,.01,safe,deadline()));
  assert(!checkedReplanSuffix(old,-.1,.5,.01,safe,deadline()));
  assert(!checkedReplanSuffix(old,1.088,.5,.000001,safe,deadline()));
  assert(!checkedReplanSuffix(old,1.088,.5,.01,safe,std::chrono::steady_clock::now()));
  assert((old.getControlPoint()-original).norm()==0.);
  // Include the endpoint, not only the regularly spaced samples.
  assert(!checkedReplanSuffix(old,1.088,.5,.07,[&](const Eigen::Vector3d &v) {
    return (v-old.evaluateDeBoorT(old.getTimeSum())).norm()>1e-10;
  },deadline()));
}
''')
    binary = tmp_path / 'checked'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3',
                    '-I', str(package / 'include'), str(source),
                    str(package / 'src/uniform_bspline.cpp'), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
