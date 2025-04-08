if [[ -z "$CONTAINER_ID" ]]; then
    alias octet="stat -c \"%a %n\""
    alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
    alias wp-keats="waypipe -c lz4=8 --video=hw ssh -Y -C keats"
    alias laservision-ipcheck="ssh laservision \"ipcheck\""
    export PATH="$HOME/.scripts:$PATH"
else
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
