#!/usr/bin/env bash
# Build generalist_bt_gen with a deterministic ROS 2 Jazzy + conda Python setup.
#
# Run from anywhere:
#   bash scripts/build_workspace.sh
#
# First-time setup helpers:
#   bash scripts/build_workspace.sh --install-colcon --install-python-deps

set -Eeuo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CONDA_ENV="${CONDA_ENV:-ros2jazzy}"
CONDA_SH="${CONDA_SH:-${HOME}/miniconda3/etc/profile.d/conda.sh}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
EXTERNAL_OVERLAYS="${EXTERNAL_OVERLAYS:-/home/luke/okvis_ws/install/setup.bash}"

BUILD_TYPE="RelWithDebInfo"
INSTALL_COLCON=0
INSTALL_PYTHON_DEPS=0
ALLOW_SYSTEM_COLCON=0
SKIP_DEP_CHECK=0
SOURCE_EXISTING_INSTALL=0
COLCON_ARGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [options] [-- extra colcon args]

Build options:
  --debug                    Build with CMAKE_BUILD_TYPE=Debug.
  --release                  Build with CMAKE_BUILD_TYPE=Release.
  --source-existing-install  Source this workspace's existing install/setup.bash before building.
  --allow-system-colcon      Fall back to /usr/bin/colcon if conda colcon is not installed.
  --skip-dep-check           Skip Python dependency import checks.

Setup helpers:
  --install-colcon           Install colcon-common-extensions into the conda env.
  --install-python-deps      Install this repo's Python runtime deps into the conda env.

Environment overrides:
  ROS_DISTRO=${ROS_DISTRO}
  CONDA_ENV=${CONDA_ENV}
  CONDA_SH=${CONDA_SH}
  WORKSPACE_DIR=${WORKSPACE_DIR}
  EXTERNAL_OVERLAYS="${EXTERNAL_OVERLAYS}"

Examples:
  bash scripts/build_workspace.sh
  bash scripts/build_workspace.sh --install-colcon --install-python-deps
  bash scripts/build_workspace.sh -- --packages-select llm_interface
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug)
      BUILD_TYPE="Debug"
      shift
      ;;
    --release)
      BUILD_TYPE="Release"
      shift
      ;;
    --source-existing-install)
      SOURCE_EXISTING_INSTALL=1
      shift
      ;;
    --allow-system-colcon)
      ALLOW_SYSTEM_COLCON=1
      shift
      ;;
    --skip-dep-check)
      SKIP_DEP_CHECK=1
      shift
      ;;
    --install-colcon)
      INSTALL_COLCON=1
      shift
      ;;
    --install-python-deps)
      INSTALL_PYTHON_DEPS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      COLCON_ARGS+=("$@")
      break
      ;;
    *)
      COLCON_ARGS+=("$1")
      shift
      ;;
  esac
done

fail() {
  echo "error: $*" >&2
  exit 1
}

source_if_present() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck source=/dev/null
    set +u
    source "${setup_file}"
    set -u
    echo "sourced ${setup_file}"
  else
    echo "skipped missing overlay ${setup_file}"
  fi
}

require_python312() {
  python - <<'PY'
import sys

if sys.version_info[:2] != (3, 12):
    raise SystemExit(
        f"expected Python 3.12 for ROS 2 Jazzy, got {sys.version.split()[0]} at {sys.executable}"
    )
print(f"using Python {sys.version.split()[0]} at {sys.executable}")
PY
}

check_python_deps() {
  python - <<'PY'
build_missing = []
runtime_missing = []

for module in (
    "rclpy",
    "setuptools",
    "em",
    "colcon_core",
):
    try:
        __import__(module)
    except Exception as exc:
        build_missing.append(f"{module}: {exc}")

for module in (
    "jinja2",
    "rich",
    "langchain",
    "langchain_google_genai",
    "langchain_openai",
    "langchain_openrouter",
    "fastapi",
    "uvicorn",
):
    try:
        __import__(module)
    except Exception as exc:
        runtime_missing.append(f"{module}: {exc}")

if build_missing:
    print("Missing or broken Python build dependencies in the active environment:", flush=True)
    for item in build_missing:
        print(f"  - {item}", flush=True)
    raise SystemExit(
        "Fix the active ros2jazzy environment before building."
    )

if runtime_missing:
    print("warning: missing optional runtime Python dependencies:", flush=True)
    for item in runtime_missing:
        print(f"  - {item}", flush=True)
    print(
        "Build can continue, but affected nodes may fail at runtime. "
        "Install them manually in ros2jazzy or rerun with --install-python-deps.",
        flush=True,
    )
else:
    print("Python dependency import check passed.")
PY
}

install_python_deps() {
  python -m pip install \
    fastapi \
    uvicorn \
    jinja2 \
    rich \
    langchain \
    langchain-core \
    langchain-google-genai \
    langchain-openai \
    langchain-openrouter \
    openai \
    requests \
    pyyaml
}

install_colcon() {
  python -m pip install colcon-common-extensions
}

choose_colcon_command() {
  if python -m colcon --help >/dev/null 2>&1; then
    COLCON_CMD=(python -m colcon)
    return
  fi

  if [[ "${ALLOW_SYSTEM_COLCON}" -eq 1 ]] && command -v colcon >/dev/null 2>&1; then
    COLCON_CMD=(colcon)
    echo "warning: using system colcon. Python entry point shebangs may use /usr/bin/python3." >&2
    return
  fi

  fail "colcon is not installed in conda env '${CONDA_ENV}'. Run with --install-colcon."
}

[[ -f "${CONDA_SH}" ]] || fail "conda activation script not found: ${CONDA_SH}"
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || fail "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash"
[[ -d "${WORKSPACE_DIR}/src" ]] || fail "workspace src directory not found: ${WORKSPACE_DIR}/src"

set +u
# shellcheck source=/dev/null
source "${CONDA_SH}"
conda activate "${CONDA_ENV}"
set -u

# Keep broken or incompatible ~/.local Python packages out of this build.
export PYTHONNOUSERSITE=1
export COLCON_PYTHON_EXECUTABLE="${CONDA_PREFIX}/bin/python"
export PIP_REQUIRE_VIRTUALENV=false
# ROS generators need Ubuntu's empy module, while ament_python package builds
# need conda's setuptools. Put only an em.py shim before conda site-packages,
# then expose apt's colcon modules after conda packages.
PYTHON_SHIM_DIR="${TMPDIR:-/tmp}/generalist_bt_gen_python_shims"
CONDA_SITE_PACKAGES="$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
mkdir -p "${PYTHON_SHIM_DIR}"
[[ -f /usr/lib/python3/dist-packages/em.py ]] || fail "Ubuntu empy module not found: /usr/lib/python3/dist-packages/em.py"
ln -sf /usr/lib/python3/dist-packages/em.py "${PYTHON_SHIM_DIR}/em.py"
export PYTHONPATH="${PYTHON_SHIM_DIR}:${CONDA_SITE_PACKAGES}:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

require_python312

if [[ "${INSTALL_COLCON}" -eq 1 ]]; then
  install_colcon
fi

if [[ "${INSTALL_PYTHON_DEPS}" -eq 1 ]]; then
  install_python_deps
fi

# Source ROS after conda activation so ROS paths are present while Python stays 3.12.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
echo "sourced /opt/ros/${ROS_DISTRO}/setup.bash"

for overlay in ${EXTERNAL_OVERLAYS}; do
  source_if_present "${overlay}"
done

if [[ "${SOURCE_EXISTING_INSTALL}" -eq 1 ]]; then
  source_if_present "${WORKSPACE_DIR}/install/setup.bash"
fi

if [[ "${SKIP_DEP_CHECK}" -ne 1 ]]; then
  check_python_deps
fi

choose_colcon_command

cd "${WORKSPACE_DIR}"

echo "building ${WORKSPACE_DIR}"
echo "colcon command: ${COLCON_CMD[*]} build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DPython3_EXECUTABLE=${CONDA_PREFIX}/bin/python ${COLCON_ARGS[*]}"

"${COLCON_CMD[@]}" build \
  --symlink-install \
  --cmake-args \
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
    "-DPython3_EXECUTABLE=${CONDA_PREFIX}/bin/python" \
  "${COLCON_ARGS[@]}"

# shellcheck source=/dev/null
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
echo "build complete; sourced ${WORKSPACE_DIR}/install/setup.bash"
