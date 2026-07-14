# syntax=docker/dockerfile:1.19

ARG CUDA_VERSION=12.6.3
FROM nvidia/cuda:${CUDA_VERSION}-cudnn-devel-ubuntu24.04

ARG ROS_DISTRO=jazzy
ARG TORCH_VERSION=2.11.0
ARG TORCHVISION_VERSION=0.26.0
ARG TORCH_INDEX_URL=https://download.pytorch.org/whl/cu126
ARG CUDA_ARCHITECTURES=86
ARG BUILD_JOBS=4

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONNOUSERSITE=1 \
    PIP_ROOT_USER_ACTION=ignore \
    CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} \
    CUDA_MODULE_LOADING=LAZY \
    PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gnupg2 \
      locales \
      software-properties-common \
    && locale-gen en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libatlas-base-dev \
      libboost-filesystem-dev \
      libcurl4-openssl-dev \
      libeigen3-dev \
      libfmt-dev \
      libgeographiclib-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      libopencv-dev \
      libopenblas-openmp-dev \
      libpcl-dev \
      libsqlite3-dev \
      libsuitesparse-dev \
      libtbb-dev \
      libtinyxml2-dev \
      libusb-1.0-0 \
      libzmq3-dev \
      nlohmann-json3-dev \
      python3-colcon-common-extensions \
      python3-pip \
      python3-pyqt5 \
      python3-rosdep \
      python3-vcstool \
      ros-${ROS_DISTRO}-ament-cmake-python \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-desktop \
      ros-${ROS_DISTRO}-eigen3-cmake-module \
      ros-${ROS_DISTRO}-generate-parameter-library \
      ros-${ROS_DISTRO}-librealsense2 \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-realsense2-camera \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-tf2-eigen \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY docker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages --no-cache-dir \
      --index-url "${TORCH_INDEX_URL}" \
      "torch==${TORCH_VERSION}" \
      "torchvision==${TORCHVISION_VERSION}" \
    && python3 -m pip install --break-system-packages --no-cache-dir \
      -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# okvis_src is supplied by Compose as ../okvis_ws/src. Excluding generated
# output keeps the build context bounded while retaining local source changes
# and the model assets required by OKVIS2-X/FindAnything.
WORKDIR /opt/ws/okvis_ws
COPY --exclude=**/.git --exclude=OKVIS2-X/build --from=okvis_src . src/

# Seed the filenames checked by OKVIS CMake from the workspace's model assets.
# This avoids re-downloading the large FindAnything models during every build.
RUN mkdir -p src/OKVIS2-X/build \
    && cp src/OKVIS2-X/resources/depth-model.pt src/OKVIS2-X/build/stereo-indoor-sigma.pt \
    && cp src/OKVIS2-X/resources/esam-model.pt src/OKVIS2-X/build/esam-model-original.pt \
    && cp src/OKVIS2-X/resources/mvs-model.pt src/OKVIS2-X/build/mvs-model.pt \
    && cp src/OKVIS2-X/resources/vl-vision-model.pt src/OKVIS2-X/build/vl-vision-model.pt

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && TORCH_CMAKE_PREFIX="$(python3 -c 'import torch; print(torch.utils.cmake_prefix_path)')" \
    && colcon build \
      --merge-install \
      --cmake-args \
        -DBUILD_ROS2=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}" \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
        -DCMAKE_PREFIX_PATH="${TORCH_CMAKE_PREFIX}" \
        -DHAVE_LIBREALSENSE=ON \
        -DUSE_COLIDMAP=ON \
        -DUSE_GPU=ON \
        -DUSE_NN=ON

WORKDIR /opt/ws/generalist_bt_gen
COPY . .

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ws/okvis_ws/install/setup.bash \
    && colcon build \
      --merge-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPython3_EXECUTABLE=/usr/bin/python3

COPY --chmod=755 docker/entrypoint.sh /ros_entrypoint.sh

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.12/dist-packages/torch/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
