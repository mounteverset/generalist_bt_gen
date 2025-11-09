#!/usr/bin/env bash
# Provision a ROS 2 Jazzy workspace that can run the generalist_bt_gen project in the cloud.
#
# The script installs ROS 2 + build tools, clones the repository, installs dependencies,
# builds the workspace, and wires up the shell environment so Codex jobs can run headlessly.
# It is idempotent and can be re-run to pick up updates.

set -Eeuo pipefail

ROS_DISTRO_DEFAULT="jazzy"
WORKSPACE_DEFAULT="${HOME}/generalist_bt_gen_ws"
REPO_URL_DEFAULT="https://github.com/luke/generalist_bt_gen.git"
BRANCH_DEFAULT="main"

ROS_DISTRO="${ROS_DISTRO:-${ROS_DISTRO_DEFAULT}}"
WORKSPACE_DIR="${WORKSPACE_DIR:-${WORKSPACE_DEFAULT}}"
REPO_URL="${REPO_URL:-${REPO_URL_DEFAULT}}"
REPO_BRANCH="${REPO_BRANCH:-${BRANCH_DEFAULT}}"
SKIP_BUILD="${SKIP_BUILD:-0}"
APPEND_BASHRC="${APPEND_BASHRC:-1}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --workspace <path>     Workspace to create/refresh (default: ${WORKSPACE_DIR})
  --repo-url <url>       Git URL for generalist_bt_gen (default: ${REPO_URL})
  --branch <name>        Git branch or tag to check out (default: ${REPO_BRANCH})
  --ros-distro <name>    ROS 2 distribution (default: ${ROS_DISTRO})
  --skip-build           Install dependencies only (skip colcon build)
  --no-bashrc            Do not modify ~/.bashrc with sourcing commands
  -h, --help             Show this message

Environment overrides:
  ROS_DISTRO, WORKSPACE_DIR, REPO_URL, REPO_BRANCH, SKIP_BUILD, APPEND_BASHRC

Notes:
  * Tested on Ubuntu 24.04 (noble) for ROS 2 Jazzy.
  * Provide OPENAI_API_KEY in the environment to persist it into the workspace secrets file.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WORKSPACE_DIR="$2"
      shift 2
      ;;
    --repo-url)
      REPO_URL="$2"
      shift 2
      ;;
    --branch)
      REPO_BRANCH="$2"
      shift 2
      ;;
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --no-bashrc)
      APPEND_BASHRC=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Command '$1' is required but not installed." >&2
    exit 1
  fi
}

ensure_ubuntu_noble() {
  local codename
  codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"
  if [[ "${codename}" != "noble" ]]; then
    echo "This script targets Ubuntu 24.04 (noble). Detected ${codename}." >&2
    exit 1
  fi
}

add_ros_apt_repo() {
  if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends curl gnupg2 lsb-release
    sudo mkdir -p /etc/apt/keyrings
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
      sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
    local arch codename
    arch="$(dpkg --print-architecture)"
    codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"
    cat <<EOF | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
deb [arch=${arch} signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main
EOF
  fi
}

install_apt_packages() {
  # Use a retrying installer to be resilient to transient mirror/network errors
  apt_install_retry \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    python3-yaml \
    libzmq3-dev \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-geographic-msgs" \
    "ros-${ROS_DISTRO}-nav-msgs" \
    "ros-${ROS_DISTRO}-nav2-msgs" \
    nlohmann-json3-dev \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-tf2-tools"
}


# Retry wrapper for apt-get installs to handle transient HTTP/mirror failures.
apt_install_retry() {
  local attempts=3
  local wait=5
  local pkgs=("$@")
  for i in $(seq 1 "$attempts"); do
    sudo apt-get update
    if sudo apt-get install -y --no-install-recommends --fix-missing "${pkgs[@]}"; then
      return 0
    fi
    echo "apt-get install failed (attempt ${i}/${attempts}). Retrying in ${wait}s..." >&2
    sleep ${wait}
    wait=$((wait * 2))
  done
  echo "apt-get install failed after ${attempts} attempts." >&2
  return 1
}

ensure_rosdep() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
}

clone_or_update_repo() {
  mkdir -p "${WORKSPACE_DIR}/src"
  local repo_dir="${WORKSPACE_DIR}/src/generalist_bt_gen"
  if [[ -d "${repo_dir}/.git" ]]; then
    git -C "${repo_dir}" fetch --all --prune
    git -C "${repo_dir}" checkout "${REPO_BRANCH}"
    git -C "${repo_dir}" pull --ff-only origin "${REPO_BRANCH}"
  else
    git clone --branch "${REPO_BRANCH}" "${REPO_URL}" "${repo_dir}"
  fi
}

install_python_packages() {
  python3 -m pip install --user --upgrade pip
  python3 -m pip install --user \
    openai \
    requests \
    pyyaml
}

build_workspace() {
  if [[ "${SKIP_BUILD}" -eq 1 ]]; then
    echo "Skipping colcon build as requested."
    return
  fi

  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  pushd "${WORKSPACE_DIR}" >/dev/null
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  popd >/dev/null
}

write_shell_snippet() {
  if [[ "${APPEND_BASHRC}" -ne 1 ]]; then
    return
  fi

  local marker="# generalist_bt_gen cloud environment"
  local snippet="${marker}
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE_DIR}/install/setup.bash"

  if ! grep -Fxq "${marker}" "${HOME}/.bashrc"; then
    {
      echo ""
      echo "${snippet}"
    } >> "${HOME}/.bashrc"
  fi
}

persist_openai_key() {
  if [[ -z "${OPENAI_API_KEY:-}" ]]; then
    return
  fi
  local secrets_dir="${WORKSPACE_DIR}/.cloud_secrets"
  mkdir -p "${secrets_dir}"
  cat > "${secrets_dir}/openai.env" <<EOF
OPENAI_API_KEY=${OPENAI_API_KEY}
EOF
  chmod 600 "${secrets_dir}/openai.env"
  echo "Stored OPENAI_API_KEY at ${secrets_dir}/openai.env"
}

main() {
  ensure_ubuntu_noble
  add_ros_apt_repo
  install_apt_packages
  ensure_rosdep
  clone_or_update_repo
  install_python_packages
  build_workspace
  write_shell_snippet
  # persist_openai_key
  echo "Cloud environment ready at ${WORKSPACE_DIR}."
  echo "To use it manually: source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash"
}

main "$@"
