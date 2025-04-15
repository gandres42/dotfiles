# ALIASES ---------------------------------------------------------------------
alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
alias wp-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"
alias ros-conda="source "${CONDA_PREFIX}/setup.bash" >/dev/null 2>&1"

# SHELL SCRIPTS ---------------------------------------------------------------
export PATH="$HOME/.dotfiles/scripts:$PATH"

# PIXI ----------------------------------------------------------------------------------
export PATH="/home/gavin/.pixi/bin:$PATH"

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
#         $HOME/.pixi/bin/pixi shell
#         echo $PIXI_PROJECT_ROOT
#         source $PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash
        bash -c 'eval "$(pixi shell-hook)"; exec bash'
    else
        $HOME/.pixi/bin/pixi "${@:1}"
    fi
}

# DISTROBOX -------------------------------------------------------------------
if [[ -n "$CONTAINER_ID" ]]; then
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
    case $CONTAINER_ID in
        "jazzy")
            source /opt/ros/jazzy/setup.bash
        ;;
        "svin")
            source /opt/ros/jazzy/setup.bash
        ;;
        "humble")
            source /opt/ros/humble/setup.bash
        ;;
        "foxy")
            source /opt/ros/foxy/setup.bash
        ;;
        "noetic")
            source /opt/ros/noetic/setup.bash
        ;;
    esac
fi

