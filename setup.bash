# ALIASES ---------------------------------------------------------------------
alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
alias wp-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"
alias wp-charybdis="waypipe --video=hw ssh -Y charybdis"
alias die="kill -9 %1"
alias c="clear"
alias re-source="source ~/.bashrc"

# SHELL SCRIPTS ---------------------------------------------------------------
export PATH="$HOME/.dotfiles/scripts:$PATH"

# PIXI ----------------------------------------------------------------------------------
export PATH="/home/gavin/.pixi/bin:$PATH"
if [[ "$TERM_PROGRAM" == "vscode" ]]; then
    export VSCODE_WORKSPACE_ROOT="$PWD"
    if [ -f pixi.lock ]; then
        eval "$($HOME/.pixi/bin/pixi shell-hook)"
        source $PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash
    elif [ -f uv.lock ]; then
        source $VSCODE_WORKSPACE_ROOT/.venv/bin/activate
    fi
    clear
fi

# FUNCTIONS -------------------------------------------------------------------
db() {
    if [ "$1" == "code" ]; then
        distrobox-code "$2"
    elif [ "$1" == "uncode" ]; then
        distrobox-uncode "$2"
    elif [ "$1" == "startall" ]; then
        distrobox-startall
    elif [ "$1" == "create" ]; then
        distrobox "${@:1}"
        distrobox-codeall
    elif [ "$1" == "uncode-all" ]; then
        rm ${HOME}/.config/Code/User/globalStorage/ms-vscode-remote.remote-containers/nameConfigs/*
    else
        distrobox "${@:1}"
    fi
}

ts() {
    if [ "$1" == "mull" ]; then
        tailscale set --exit-node=$(tailscale exit-node suggest | awk -F': ' '/Suggested exit node:/ {print $2}' | sed 's/\.$//')
        ipcheck
    elif [ "$1" == "unmull" ]; then
        tailscale set --exit-node=
        ipcheck
    else
        tailscale "${@:1}"
    fi
}

pixi() {
    if [ "$1" == "shell" ]; then
        bash --rcfile <(cat ~/.bashrc; echo 'eval "$(pixi shell-hook)"'; echo '[ -f "$PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash" ] && source "$PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash"'; echo '[[ -z "$PIXI_PROJECT_ROOT" ]] && exit 1;')
    else
        $HOME/.pixi/bin/pixi "${@:1}"
    fi
}

# DISTROBOX -------------------------------------------------------------------
if [[ -n "$CONTAINER_ID" || "$HOSTNAME" == *.* ]]; then
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
    export QT_QPA_PLATFORM=xcb
fi

# ROS -------------------------------------------------------------------------
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
if [[ "$ROS_DISTRO" == "noetic" ]]; then
    alias cbs="catkin build && source devel/setup.bash"
    alias plotjuggler="rosrun plotjuggler plotjuggler -n"
else
    alias cbs="colcon build && source install/setup.bash"
    alias plotjuggler="ros2 run plotjuggler plotjuggler -n"
fi

# ENV VARIABLES ---------------------------------------------------------------
export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification
