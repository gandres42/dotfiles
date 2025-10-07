FROM osrf/ros:noetic-desktop-full
ARG USERNAME=USERNAME
ARG CONTAINER_ID=CONTAINER_ID
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# create non-root user and add to sudo
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# install uv
RUN apt-get update && apt-get install curl && curl -LsSf https://astral.sh/uv/install.sh | UV_INSTALL_DIR="/usr/local/bin" sh
ENV UV_SYSTEM_PYTHON=1
ENV UV_BREAK_SYSTEM_PACKAGES=true
RUN echo 'Defaults env_keep += "UV_SYSTEM_PYTHON UV_BREAK_SYSTEM_PACKAGES"' >> /etc/sudoers.d/$USERNAME

# update and install apt packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    python3-pip \
    python-is-python3 \
    git \
    python3-catkin-tools \
    wget \
    libompl-dev \
    libc++-dev \
    libc++abi-dev \
    ros-noetic-octomap-ros \
    ros-noetic-grid-map-costmap-2d

# install open3d c++
RUN cd /tmp \
    && wget https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz \
    && tar -xf open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz \
    && cp -r open3d-devel-linux-x86_64-cxx11-abi-0.19.0/include/* /usr/include \
    && cp -r open3d-devel-linux-x86_64-cxx11-abi-0.19.0/lib/* /usr/lib \
    && cp -r open3d-devel-linux-x86_64-cxx11-abi-0.19.0/share/* /usr/share

# install python packages
RUN uv pip install \
    open3d \
    filterpy

# dev environemnt setup
RUN chsh -s /bin/bash $USERNAME
RUN apt install -y jq
USER $USERNAME
WORKDIR /home/$USERNAME
# RUN git clone https://github.com/gandres42/dotfiles.git && mv dotfiles .dotfiles
RUN echo 'source ~/.dotfiles/setup.bash' >> ~/.bashrc
RUN echo 'alias uv="sudo uv"' >> ~/.bashrc
ENV CONTAINER_ID $CONTAINER_ID