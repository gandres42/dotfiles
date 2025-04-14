# ALIASES
alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
alias wp-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"

# DISTROBOX
if [[ -n "$CONTAINER_ID" ]]; then
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
    case $CONTAINER_ID in
        "jazzy")
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

# MAMBA
if [[ "$TERM_PROGRAM" != "vscode" ]]; then
    __conda_setup="$('/home/gavin/.miniforge/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
    if [ $? -eq 0 ]; then
        eval "$__conda_setup"
    else
        if [ -f "/home/gavin/.miniforge/etc/profile.d/conda.sh" ]; then
            . "/home/gavin/.miniforge/etc/profile.d/conda.sh"
        else
            export PATH="/home/gavin/.miniforge/bin:$PATH"
        fi
    fi
    unset __conda_setup

    if [ -f "/home/gavin/.miniforge/etc/profile.d/mamba.sh" ]; then
        . "/home/gavin/.miniforge/etc/profile.d/mamba.sh"
    fi
fi

# PIXI
export PATH="/home/gavin/.pixi/bin:$PATH"
if [[ "$TERM_PROGRAM" == "vscode" ]]; then
    eval "$(pixi shell-hook)" >/dev/null 2>&1
    if [ -n "$CONDA_PREFIX" ]; then
        source "${CONDA_PREFIX}/setup.bash" >/dev/null 2>&1
    fi
fi

# SHELL SCRIPTS
export PATH="$HOME/.dotfiles/scripts:$PATH"

# FUNCTIONS
if [ -d $HOME/.dotfiles/functions ]; then
    for file in $HOME/.dotfiles/functions/*; do
        [ -f "$file" ] && source "$file"
    done
fi
