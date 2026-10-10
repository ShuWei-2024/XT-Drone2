# Legacy Sophus dependency

Source: https://github.com/strasdat/Sophus, commit
`a621ff2e56c56c839a6c40418d42c3c254424b5c`.

This is the non-template Sophus API used by the current FAST-LIVO2/vikit port.
The existing local `SO2` constructor fix uses `std::complex::real(value)` and
`imag(value)` for modern C++ compatibility. Original source copyright and
license notices remain in the files.

`CMakeLists.upstream.txt` preserves the original build configuration.
The new CMake file builds the same six library sources and exports the legacy
variables without the removed CMake `LOCATION` behavior. `build_sophus_117`
holds generated files and is ignored by Git. New runtime uses this directory,
not the old ROS workspace's Sophus build or installation.
