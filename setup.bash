# region: SHELL SCRIPTS -------------------------------------------------------

export PATH="$HOME/.dotfiles/scripts:$PATH"
export PPID_NAME=$(ps -o comm= $(ps -o ppid= -p $$))

# endregion

# region: ALIASES -------------------------------------------------------------

alias wake-keats="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
alias c="clear"
alias re-source="source ~/.bashrc"
alias get-mit="wget https://www.mit.edu/~amini/LICENSE.md"
alias ipcheck="curl -s http://ip-api.com/json/ | jq"
alias dotfile-edit="codium $HOME/.dotfiles"
alias beemovie="curl -sSL https://gist.githubusercontent.com/MattIPv4/045239bc27b16b2bcf7a3a9a4648c08a/raw/2411e31293a35f3e565f61e7490a806d4720ea7e/bee%2520movie%2520script"
alias smi="watch -t -n 0.1 nvidia-smi"
alias open3d-stubs='pybind11-stubgen -o $(python -c "import site; print(site.getsitepackages()[0])") --root-suffix "" open3d'
alias vault-setup='ln -s ../.attachments Attachments && ln -s ../.obsidian .obsidian'
alias xpra-server="xpra start :100 --daemon=no --bind-tcp=0.0.0.0:15000"
alias betterfox="wget https://raw.githubusercontent.com/yokoffing/Betterfox/main/user.js"
alias db="distrobox"

# endregion

# region: PKILL ---------------------------------------------------------------

hitman() {
    local pid
    pid=$(jobs -p %1) || return 1

    kill -9 -- -"$pid" 2>/dev/null   # kill entire process group

    while kill -0 "$pid" 2>/dev/null; do
        sleep 0.05
    done
    echo "excellent work, 47"
}

# endregion

# region: DEVCONTAINERS -------------------------------------------------------

dcr() {
    if [ $# -ne 1 ]; then
        echo "Usage: dcr <base-image>"
        return 1
    fi
    local wsname="$(basename "$PWD")"
    local baseimage="$1"
    cp -r "$HOME/.dotfiles/devcontainer" .devcontainer
    find .devcontainer -type f -exec sed -i "s|WSNAME|$wsname|g" {} +
    find .devcontainer -type f -exec sed -i "s|BASEIMAGE|$baseimage|g" {} +
}

# endregion

# region: DISTROBOX -----------------------------------------------------------

if [[ -n "$CONTAINER_ID" ]]; then
#     PS1="📦 $PS1"
    export QT_QPA_PLATFORM=xcb
    [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
    [ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
    [ -f /opt/ros/kilted/setup.bash ] && source /opt/ros/kilted/setup.bash
fi

# endregion

# region: IDE AUTO-ACTIVATION -------------------------------------------------

IDE_PROGRAMS=("code" "pycharm" "codium" "clion" "vscode" "zed")

if [[ " ${IDE_PROGRAMS[@]} " =~ " ${PPID_NAME} " || " ${IDE_PROGRAMS[@]} " =~ " ${TERM_PROGRAM} " || -n "$PIXI_ACTIVATE" ]]; then
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
    elif [ -e $dir/.condaenv ]; then
        mamba activate $(cat .condaenv)
    fi
    clear

    export XDG_SESSION_TYPE=x11
fi

# endregion

# region: PIXI ----------------------------------------------------------------

if [[ -e "$HOME/.pixi" ]]; then
    export PATH="/home/gavin/.pixi/bin:$PATH"
    pixi() {
        if [ "$1" == "shell" ]; then
            bash --rcfile <(echo 'eval "$(pixi shell-hook --change-ps1 false)"'; cat ~/.bashrc; echo '[ -f "$PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash" ] && source "$PIXI_PROJECT_ROOT/.pixi/envs/default/setup.bash"'; echo '[[ -z "$PIXI_PROJECT_ROOT" ]] && exit 1;'; echo 'export PS1="($PIXI_PROJECT_NAME) $PS1"')
        elif [ "$1" == "init" ]; then
            $HOME/.pixi/bin/pixi "${@:1}"
        elif [ "$1" == "pip" ]; then
            if [ -n "$PIXI_PROJECT_NAME" ]; then
                $HOME/.local/bin/uv "${@:1}" --system
            else
                echo -e "Error:   \e[31m×\e[0m enter a pixi shell to use pip"
            fi
        elif [ "$1" == "ros" ]; then
            if ! pixi list &>/dev/null; then
                pixi init
            fi
            if [ "$2" == "noetic" ]; then
                pixi project channel add robostack-noetic
                pixi add ros-noetic-desktop
            elif [[ "$2" == "jazzy" || "$2" == "kilted" || "$2" == "humble" ]] || [[ -z "$2" ]]; then
                pixi project channel add robostack-"$2"
                pixi add ros-"$2"-desktop ros-"$2"-foxglove-bridge ros-"$2"-rmw-zenoh-cpp colcon-common-extensions
            else
                echo "Error: Unsupported ROS distro '$2'. Supported: noetic, humble, jazzy, kilted"
                return 1
            fi
        else
            $HOME/.pixi/bin/pixi "${@:1}"
        fi
    }
fi

# endregion

# region: TAILSCALE  ----------------------------------------------------------

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

# endregion

# region: ROS -----------------------------------------------------------------

unalias cbs 2>/dev/null

if [[ "$ROS_DISTRO" == "noetic" ]]; then
    function cbs {
        catkin build "$@" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-isystem /opt/ros/noetic/include' && source devel/setup.bash && jq -s 'add' build/*/compile_commands.json > compile_commands.json
    }
    alias s="source devel/setup.bash"
    alias plotjuggler="rosrun plotjuggler plotjuggler -n"
    export DISABLE_ROS1_EOL_WARNINGS=1
    export CMAKE_POLICY_VERSION_MINIMUM=3.5
elif [[ -n "$ROS_DISTRO" ]]; then
    function cbs {
        colcon build "$@" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash && ls build/*/compile_commands.json >/dev/null 2>&1 && jq -s 'add' build/*/compile_commands.json > compile_commands.json
    }

    alias roscore="ros2 run rmw_zenoh_cpp rmw_zenohd"
    alias rosbasics="sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp ros-$ROS_DISTRO-foxglove-bridge"
    alias s="source install/setup.bash"
    alias plotjuggler="ros2 run plotjuggler plotjuggler -n"
    alias foxglove="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
    alias foxglove-remote="ros2 launch foxglove_bridge foxglove_bridge_launch.xml use_compression:=true"
    export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export QT_QUICK_CONTROLS_MATERIAL_THEME=Dark
    export QT_QUICK_CONTROLS_MATERIAL_PRIMARY=\#303030
    export QT_QUICK_CONTROLS_MATERIAL_ACCENT=Orange
fi

# endregion

# region: VARIABLES -----------------------------------------------------------

export UV_NO_BUILD_ISOLATION=true
export UV_PYTHON_PREFERENCE="system"

# endregion -------------------------------------------------------------------
