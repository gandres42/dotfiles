# ALIASES
alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
alias wp-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"
alias ros-conda="source "${CONDA_PREFIX}/setup.bash" >/dev/null 2>&1"

# SHELL SCRIPTS
export PATH="$HOME/.dotfiles/scripts:$PATH"

# FUNCTIONS
if [ -d $HOME/.dotfiles/functions ]; then
    for file in $HOME/.dotfiles/functions/*; do
        [ -f "$file" ] && source "$file"
    done
fi

# DISTROBOX
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
