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
    export PATH=$PATH:~/.scripts
elif [[ "$CONTAINER_ID" == "humble" ]]; then
    source /opt/ros/humble/setup.bash
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
    export ROS_DOMAIN_ID=5
    export ROS_DISCOVERY_SERVER="10.214.154.212:11811"
elif [[ "$CONTAINER_ID" == "noetic" ]]; then
    source /opt/ros/noetic/setup.bash
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
fi
alias wake-severn="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""

export SDKMAN_DIR="$HOME/.sdkman"
[[ -s "$HOME/.sdkman/bin/sdkman-init.sh" ]] && source "$HOME/.sdkman/bin/sdkman-init.sh"
