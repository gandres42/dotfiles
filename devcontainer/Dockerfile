FROM osrf/ros:jazzy-desktop-full
ARG USERNAME=USERNAME
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

# update sources and install basic tools
RUN apt-get update && apt-get upgrade -y
RUN apt-get install git wget python3-pip python-is-python3 jq

# install uv
RUN apt-get update && apt-get install curl && curl -LsSf https://astral.sh/uv/install.sh | UV_INSTALL_DIR="/usr/local/bin" sh
ENV UV_SYSTEM_PYTHON=1
ENV UV_BREAK_SYSTEM_PACKAGES=true
RUN echo 'Defaults env_keep += "UV_SYSTEM_PYTHON UV_BREAK_SYSTEM_PACKAGES"' >> /etc/sudoers.d/$USERNAME

# dev environemnt setup
RUN chsh -s /bin/bash $USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME
RUN echo 'source ~/.dotfiles/setup.bash' >> ~/.bashrc && echo 'alias uv="sudo uv"' >> ~/.bashrc