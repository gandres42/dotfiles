# .bashrc

# Source global definitions
if [ -f /etc/bashrc ]; then
    . /etc/bashrc
fi

# User specific environment
if ! [[ "$PATH" =~ "$HOME/.local/bin:$HOME/bin:" ]]; then
    PATH="$HOME/.local/bin:$HOME/bin:$PATH"
fi
export PATH

# Uncomment the following line if you don't like systemctl's auto-paging feature:
# export SYSTEMD_PAGER=

# User specific aliases and functions
if [ -d ~/.bashrc.d ]; then
    for rc in ~/.bashrc.d/*; do
        if [ -f "$rc" ]; then
            . "$rc"
        fi
    done
fi
unset rc

if [[ -z "$CONTAINER_ID" ]]; then
    alias octet="stat -c \"%a %n\""
    alias speed="speedtest-cli"
    alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
    alias waypipe-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"
    export SDKMAN_DIR="$HOME/.sdkman"
    [[ -s "$HOME/.sdkman/bin/sdkman-init.sh" ]] && source "$HOME/.sdkman/bin/sdkman-init.sh"
    export PATH=$PATH:~/.scripts
else
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
    case $CONTAINER_ID in
        "humble")
            source /opt/ros/humble/setup.bash
        ;;
        "noetic")
            source /opt/ros/noetic/setup.bash
        ;;
	"orbslam")
            source /opt/ros/noetic/setup.bash
        ;;
        "rdml")
            source /opt/ros/noetic/setup.bash
            export ROS_MASTER_URI=http://keats:11311/
        ;;
    esac
fi
