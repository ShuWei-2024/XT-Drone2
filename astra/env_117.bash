# Source from a clean terminal. This file does not launch or arm PX4.
_astra_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${AMENT_PREFIX_PATH:-}" == *astra_ros2_ws* || "${PYTHONPATH:-}" == *astra_ros2_ws* ]]; then
    echo 'Old Astra overlay detected; open a clean terminal before loading 1.17.' >&2
    unset _astra_repo_dir
    return 1
fi
if [[ ! -f "$_astra_repo_dir/install_117/local_setup.bash" ]]; then
    echo 'Build install_117 first; see README.md.' >&2
    unset _astra_repo_dir
    return 1
fi
source /opt/ros/jazzy/setup.bash
source "$_astra_repo_dir/install_117/local_setup.bash"
unset _astra_repo_dir
