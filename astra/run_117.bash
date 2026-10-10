#!/usr/bin/env bash
set -eo pipefail
astra_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$astra_repo_dir/astra/env_117.bash"
export ASTRA_117_ENV_LOADED=1
python3 "$astra_repo_dir/astra/tools/run_sitl_117.py" "$@"
