"""Reject volume/line covariance that passes an absolute plane spread cutoff."""
from pathlib import Path
import subprocess


def test_plane_volume_line_and_scale(tmp_path):
    include=Path(__file__).resolve().parents[1]/'packages/astra_fast_livo2_core/include'
    source=tmp_path/'quality.cpp'
    source.write_text('''
#include "plane_quality.h"
#include <cassert>
#include <limits>
int main() {
  // A compact 3D cluster passes the old absolute threshold, but is not a plane.
  assert(plane_spectrum_accepted(.002,.003,.004,.005,0.,0.));
  assert(!plane_spectrum_accepted(.002,.003,.004,.005,.1,.01));
  // A line's transverse covariance cannot establish a reliable plane normal.
  assert(!plane_spectrum_accepted(1e-7,1e-6,.02,.005,.1,.01));
  assert(plane_spectrum_accepted(.00001,.002,.02,.005,.1,.01));
  for (double scale : {.1,1.,10.}) {
    assert(plane_spectrum_accepted(.00001*scale,.002*scale,.02*scale,.005*scale,.1,.01));
    assert(!plane_spectrum_accepted(.002*scale,.003*scale,.004*scale,.005*scale,.1,.01));
  }
  assert(!plane_spectrum_accepted(0.,0.,0.,.005,.1,.01));
  assert(!plane_spectrum_accepted(0.,.001,std::numeric_limits<double>::infinity(),.005,.1,.01));
  Eigen::Vector3d values;
  Eigen::Matrix3d vectors;
  assert(!ordered_plane_spectrum(Eigen::Matrix3d::Zero(),values,vectors));
  assert(!ordered_plane_spectrum(Eigen::Matrix3d::Identity(),values,vectors));
  Eigen::Matrix3d covariance = Eigen::Vector3d(.02,.00001,.002).asDiagonal();
  assert(ordered_plane_spectrum(covariance,values,vectors));
  assert((values-Eigen::Vector3d(.00001,.002,.02)).norm()<1e-12);
  assert((covariance*vectors-vectors*values.asDiagonal()).norm()<1e-12);
  assert(!ordered_plane_spectrum(Eigen::Vector3d(0.,0.,.02).asDiagonal(),values,vectors));
}
''')
    binary=tmp_path/'quality'
    subprocess.run(['g++','-std=c++17','-include','initializer_list','-I/usr/include/eigen3','-I',str(include),
                    str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
