#!/usr/bin/env bash
# Source this file from a terminal: source Tools/setup/xtdrone2_env.sh
# Supports Ubuntu 24.04, ROS 2 Jazzy and the user's Python virtual environment.
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    printf 'ROS 2 Jazzy is not installed in /opt/ros/jazzy.\n' >&2
    return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$HOME/xtd2_pyenv/bin/activate" ]]; then
    printf 'Missing Python virtual environment: %s/xtd2_pyenv\n' "$HOME" >&2
    return 1 2>/dev/null || exit 1
fi
source "$HOME/xtd2_pyenv/bin/activate"
# ROS Jazzy supplies Gazebo Harmonic through its vendor packages.
if [[ -d /opt/ros/jazzy/opt/gz_tools_vendor/bin ]]; then
    export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"
fi
# Keep the supported XRCE Agent separate from any system or v3 installation.
if [[ -d "$HOME/.local/px4-xrce-agent-v2.4.3/bin" ]]; then
    export PATH="$HOME/.local/px4-xrce-agent-v2.4.3/bin:$PATH"
fi
