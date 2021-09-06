FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04 AS base

ENV DEBIAN_FRONTEND noninteractive
RUN ["/bin/bash", "-c", "set -o pipefail && echo 'APT::Install-Recommends \"0\";'> /etc/apt/apt.conf.d/01satinstall"]

RUN apt-get update && apt-get install -y curl \
                                         wget \
                                         gnupg2 \
                                         lsb-release \
                                         python3-argcomplete \
                                         locales \
                                && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8

# Add SAT mirror key
RUN ["/bin/bash", "-c", "set -o pipefail && curl -s https://mirror.sberautotech.ru/apt/mirror.pgp.key | apt-key add -"]

# Override original Nvidia repos to SAT mirrors
RUN rm /etc/apt/sources.list.d/*.list
RUN ["/bin/bash", "-c", "set -o pipefail && echo \"deb [arch=$(dpkg --print-architecture)] https://mirror.sberautotech.ru/apt/sat-nvidia-cuda-ubuntu2004 focal main\" > /etc/apt/sources.list.d/sat-nvidia-cuda.list"]
RUN ["/bin/bash", "-c", "set -o pipefail && echo \"deb [arch=$(dpkg --print-architecture)] https://mirror.sberautotech.ru/apt/sat-nvidia-machine-learning-ubuntu2004 focal main\" > /etc/apt/sources.list.d/sat-nvidia-machine-learning.list"]

RUN apt-get update && apt-get install -y pkg-config \
                                         libglvnd-dev \
                                         libgl1-mesa-dev \
                                         libegl1-mesa-dev \
                                         libgles2-mesa-dev \
                                && rm -rf /var/lib/apt/lists/*

# Set driver caps to all
# It seems to be fine to set all caps: https://github.com/NVIDIA/nvidia-container-runtime/issues/112
ENV NVIDIA_DRIVER_CAPABILITIES all

# ROS & ROS2 repos
RUN ["/bin/bash", "-c", "set -o pipefail && echo \"deb [arch=$(dpkg --print-architecture)] https://mirror.sberautotech.ru/apt/sat-ros $(lsb_release -cs) main\" > /etc/apt/sources.list.d/sat-ros.list"]
RUN ["/bin/bash", "-c", "set -o pipefail && echo \"deb [arch=$(dpkg --print-architecture)] https://mirror.sberautotech.ru/apt/sat-ros2 $(lsb_release -cs) main\" > /etc/apt/sources.list.d/sat-ros2.list"]

# Add sat-3rdparty
RUN ["/bin/bash", "-c", "set -o pipefail && echo \"deb [arch=$(dpkg --print-architecture)] https://mirror.sberautotech.ru/apt/sat-3rdparty $(lsb_release -cs) main\" > /etc/apt/sources.list.d/sat-3rdparty.list"]

COPY 99sat-repository /etc/apt/preferences.d/99sat-repository

FROM base AS bin
RUN apt-get update && apt-get install -y python3-pip \
                                         python3-gst-1.0 \
                                         python3-rosdep \
                                         ros-foxy-desktop \
                                         libgstreamer1.0-0 \
                                         gstreamer1.0-plugins-base \
                                         gstreamer1.0-plugins-good \
                                         gstreamer1.0-plugins-bad \
                                         gstreamer1.0-plugins-ugly \
                                         gstreamer1.0-libav \
                                         gstreamer1.0-doc \
                                         gstreamer1.0-tools \
                                         gstreamer1.0-x \
                                         gstreamer1.0-alsa \
                                         gstreamer1.0-gl \
                                         gstreamer1.0-gtk3 \
                                         gstreamer1.0-qt5 \
                                         gstreamer1.0-pulseaudio \
                                         gir1.2-gstreamer-1.0 \
                                         libgstrtspserver-1.0 \
                                         gstreamer1.0-rtsp \
                                         v4l-utils \
                                         iproute2 \
                                         libyaml-cpp0.6 \
                                         ffmpeg \
                                         qt5-style-kvantum qt5-style-kvantum-themes \
                                         libfontconfig1 \
                                         x11-utils \
                        && rm -rf /var/lib/apt/lists/*

FROM bin AS dev
RUN apt-get update && apt-get install -y python3-dev \
                                         python3-colcon-common-extensions \
                                         telnet \
                                         traceroute \
                                         iputils-ping \
                                         libyaml-cpp-dev \
                                         libgstrtspserver-1.0-dev \
                                         git \
                                         xpra \
                        && rm -rf /var/lib/apt/lists/*

# Install gst-rst
ARG GST_ARCHIVE=gst-rtsp-dev.tar.gz
ARG GST_URL=https://gitlab.sberautotech.ru/avstolbov/gst-rtsp/-/archive/dev/gst-rtsp-dev.tar.gz
RUN wget -q --show-progress --progress=bar:force:noscroll 2>&1 ${GST_URL} \
    && tar -xzf ${GST_ARCHIVE} \
    && cd gst-rtsp-dev \
    && mkdir build && cd build && cmake .. && make \
    && mv gst-rtsp /usr/local/bin/gst-rtsp \
    && chmod 755 /usr/local/bin/gst-rtsp \
    && cd ../.. && rm -rf gst-rtsp-dev \
    && rm ${GST_ARCHIVE}
# Add gst-rst to path
ENV PATH="/usr/local/bin:${PATH}"


FROM dev AS debug
ARG host_docker_group_id
RUN apt-get update && apt-get install -y bash-completion \
                                         htop \
                                         nano \
                                         less \
                                         sat-host-utils \
                                     && rm -rf /var/lib/apt/lists/*
RUN if [ ! -z "$host_docker_group_id" ] ; then groupadd -g $host_docker_group_id -o docker ; else echo "No docker group id arg, are we in CI?" ; fi

ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3 \
    ROS_DISTRO=foxy
RUN . /opt/ros/foxy/setup.sh