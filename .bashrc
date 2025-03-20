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
    alias wp-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"
    alias wp-leadbelly="waypipe -c lz4=8 ssh -Y -C leadbelly.engr.oregonstate.edu"
    alias laservision-ipcheck="ssh laservision \"ipcheck\""
    export SDKMAN_DIR="$HOME/.sdkman"
    [[ -s "$HOME/.sdkman/bin/sdkman-init.sh" ]] && source "$HOME/.sdkman/bin/sdkman-init.sh"
    export PATH="$HOME/.scripts:$PATH"
else
    PS1="📦[\u@${CONTAINER_ID} \W]\$ "
    case $CONTAINER_ID in
        "humble")
            source /opt/ros/humble/setup.bash
        ;;
	"exmax")
            source /opt/ros/foxy/setup.bash
	    export PYTHONPATH="/home/gavin/Git/em_exploration/build:$PYTHONPATH"
	    export TURTLEBOT3_MODEL=waffle
	    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models 
        ;;
        "noetic")
            source /opt/ros/noetic/setup.bash
        ;;
    esac
fi
