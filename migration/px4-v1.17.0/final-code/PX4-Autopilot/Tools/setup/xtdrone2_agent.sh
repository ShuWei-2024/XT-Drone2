#!/usr/bin/env bash
# Install the XRCE Agent version compatible with PX4 v1.17's default v2 client.
set -e
source "$(dirname "${BASH_SOURCE[0]}")/xtdrone2_env.sh"
agent_source="$HOME/Micro-XRCE-DDS-Agent-v2.4.3"
agent_prefix="$HOME/.local/px4-xrce-agent-v2.4.3"
if [[ ! -d "$agent_source/.git" ]]; then
    git clone --branch v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_source"
fi
if [[ "$(git -C "$agent_source" rev-parse HEAD)" != 73622810d984349b80bbac0ef55fc0b694d62222 ]]; then
    printf 'Agent source is not the expected v2.4.3 commit.\n' >&2
    exit 1
fi
cmake -S "$agent_source" -B "$agent_source/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$agent_prefix" \
    -DCMAKE_INSTALL_RPATH="$agent_prefix/lib;/opt/ros/jazzy/lib" \
    -DUAGENT_SUPERBUILD=ON \
    -DUAGENT_P2P_PROFILE=OFF \
    -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON \
    -DUAGENT_USE_SYSTEM_LOGGER=OFF
cmake --build "$agent_source/build" --parallel 1
cmake --install "$agent_source/build"
printf 'Agent installed: %s/bin/MicroXRCEAgent\n' "$agent_prefix"
