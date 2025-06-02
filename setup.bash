# ALIASES ---------------------------------------------------------------------
alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
alias wp-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"
alias wp-charybdis="waypipe --video=hw ssh -Y charybdis"
alias die="kill -9 %1"
alias c="clear"
alias re-source="source ~/.bashrc"
alias get-mit="wget https://www.mit.edu/~amini/LICENSE.md"

# SHELL SCRIPTS ---------------------------------------------------------------
export PATH="$HOME/.dotfiles/scripts:$PATH"

# PIXI INCLUDE ------------------------------------------------------------------
export PATH="/home/gavin/.pixi/bin:$PATH"

# DISTROBOX -------------------------------------------------------------------
if [[ -n "$CONTAINER_ID" || "$HOSTNAME" == *.* ]]; then
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
    export QT_QPA_PLATFORM=xcb
    [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
    [ -f /opt/ros/kilted/setup.bash ] && source /opt/ros/kilted/setup.bash
fi

# VSCODE AUTO-ACTIVATION ------------------------------------------------------
if [[ "$TERM_PROGRAM" == "vscode" ]]; then
    dir="$PWD"
    while [[ "$dir" != "/" ]]; do
        if [[ -d "$dir/.pixi" || -d "$dir/.venv" ]]; then
            break
        fi
        dir="$(dirname "$dir")"
    done
    export VSCODE_WORKSPACE_ROOT="$dir"
    if [ -d $dir/.pixi ]; then
        eval "$($HOME/.pixi/bin/pixi shell-hook)"
        if [ -f $PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash ]; then
            source $PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash
        fi
    elif [ -d $dir/.venv ]; then
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

function ls() {
  if [ "$PWD" = "$HOME" ]; then
    command ls --hide=snap "$@"
  else
    command ls "$@"
  fi
}

pixi() {
    if [ "$1" == "shell" ]; then
        bash --rcfile <(echo 'eval "$(pixi shell-hook)"'; cat ~/.bashrc; echo '[ -f "$PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash" ] && source "$PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash"'; echo '[[ -z "$PIXI_PROJECT_ROOT" ]] && exit 1;'; echo 'export PS1="($PIXI_PROJECT_NAME) $PS1"')
    elif [ "$1" == "pip" ]; then
        if [ -n "$PIXI_PROJECT_NAME" ]; then
            $HOME/.local/bin/uv "${@:1}" --system
        else
            echo -e "Error:   \e[31m×\e[0m could not find pixi.toml or pyproject.toml at directory $PWD"
        fi
    else
        $HOME/.pixi/bin/pixi "${@:1}"
    fi
}

# ROS -------------------------------------------------------------------------
if [[ "$ROS_DISTRO" == "noetic" ]]; then
    alias cbs="catkin build && source devel/setup.bash"
    alias s="source devel/setup.bash"
    alias plotjuggler="rosrun plotjuggler plotjuggler -n"
else
    alias cbs="colcon build && source install/setup.bash"
    alias s="source install/setup.bash"
    alias plotjuggler="ros2 run plotjuggler plotjuggler -n"
fi

# ENV VARIABLES ---------------------------------------------------------------
export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification

