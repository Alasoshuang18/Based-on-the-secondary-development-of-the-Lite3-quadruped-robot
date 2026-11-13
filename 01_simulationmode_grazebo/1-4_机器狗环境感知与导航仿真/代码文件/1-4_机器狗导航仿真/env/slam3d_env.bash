# Explicit, process-local 3-D overlay. The original 2-D environment stays intact.
_nav3d_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_nav3d_root/env/cli_env.bash" || return 1
_nav3d_prefix="$_nav3d_root/.runtime/slam3d_root/opt/ros/humble"
if [[ ! -d "$_nav3d_prefix/share/ament_index" ]]; then
  echo 'Run 6-4/install_slam3d_private.py first; no system install is needed.' >&2
  unset _nav3d_root _nav3d_prefix
  return 1
fi
export AMENT_PREFIX_PATH="$_nav3d_prefix:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$_nav3d_prefix/lib:$_nav3d_prefix/lib/aarch64-linux-gnu:$_nav3d_root/.runtime/slam3d_root/usr/lib/aarch64-linux-gnu:$_nav3d_root/.runtime/slam3d_root/usr/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$_nav3d_prefix/local/lib/python3.10/dist-packages:$_nav3d_prefix/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
unset _nav3d_root _nav3d_prefix
