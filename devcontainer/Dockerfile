FROM BASEIMAGE
ARG USERNAME=USERNAME
ARG CONTAINER_ID=CONTAINER_ID
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# create non-root user and add to sudo
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# update and install basic tools
RUN apt-get upgrade -y
RUN apt-get install -y ssh curl git wget python3-pip python-is-python3 jq

# dev environment setup
RUN chsh -s /bin/bash $USERNAME
USER $USERNAME
RUN echo "source /home/$USERNAME/.dotfiles/setup.bash" >> ~/.bashrc && echo 'export PIP_BREAK_SYSTEM_PACKAGES=1' >> ~/.bashrc
ENV CONTAINER_ID $CONTAINER_ID