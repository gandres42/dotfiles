# region: SHELL SCRIPTS -------------------------------------------------------

export PATH="$HOME/.dotfiles/scripts:$PATH"
export PPID_NAME=$(ps -o comm= $(ps -o ppid= -p $$))

# endregion

# region: ALIASES -------------------------------------------------------------

alias wake-severn="ssh discovision \"wakeonlan D8:5E:D3:D9:EF:E4\""
alias fp="flatpak --user"
alias die="kill -9 %1"
alias c="clear"
alias re-source="source ~/.bashrc"
alias get-mit="wget https://www.mit.edu/~amini/LICENSE.md"
alias ipcheck="curl -s http://ip-api.com/json/ | jq"
alias dotfile-edit="code $HOME/.dotfiles"
alias beemovie="curl -sSL https://gist.githubusercontent.com/MattIPv4/045239bc27b16b2bcf7a3a9a4648c08a/raw/2411e31293a35f3e565f61e7490a806d4720ea7e/bee%2520movie%2520script"
alias smi="watch -t -n 0.1 nvidia-smi"

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

# endregion

# region: IDE AUTO-ACTIVATION -------------------------------------------------

IDE_PROGRAMS=("code" "pycharm" "codium" "clion" "vscode")

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
                pixi add ros-"$2"-desktop ros-"$2"-foxglove-bridge ros-"$2"-rmw-zenoh-cpp
            else
                echo "Error: Unsupported ROS distro '$2'. Supported: noetic, humble, jazzy, kilted"
                return 1
            fi
        else
            $HOME/.pixi/bin/pixi "${@:1}"
        fi
    }
fi

# region: UV ------------------------------------------------------------------

if [[ -e "$HOME/.local/bin/uv" ]]; then
    uv() {
        if [ "$1" == "shell" ]; then
            p="$(uv run python -c 'import sys; print(sys.prefix)')" && [ "$p" = "/usr" ] || source "$p/bin/activate"
        else
            $HOME/.local/bin/uv "${@:1}"
        fi
    }
fi

# endregion

# endregion

# region: TAILSCALE  ----------------------------------------------------------

ts() {
    if [ "$1" == "mull" ]; then
        tailscale set --exit-node=$(tailscale exit-node suggest | awk -F': ' '/Suggested exit node:/ {print $2}' | sed 's/\.$//')
        ipcheck
    elif [ "$1" == "unmull" ]; then
        tailscale set --exit-node=
        ipcheck
    elif [ "$1" == "tar" ]; then
        if [ "$2" == "cp" ]; then
            if [ $# -lt 4 ]; then
                echo "Usage: ts tar cp <file-or-directory> <hostname>:"
                return 1
            fi
            local source="$3"
            local hostname="$4"
            
            # Validate hostname ends with colon
            if [[ ! "$hostname" =~ :$ ]]; then
                echo "Error: hostname must end with ':'"
                echo "Usage: ts tar cp <file-or-directory> <hostname>:"
                return 1
            fi
            
            local basename=$(basename "$source")
            local archive="/tmp/${basename}.tar.gz"
            
            echo "Creating archive: $archive"
            tar -czf "$archive" -C "$(dirname "$source")" "$(basename "$source")"
            
            echo "Copying to $hostname"
            ts file cp "$archive" "${hostname}"
            
            echo "Cleaning up local archive"
            rm "$archive"
        elif [ "$2" == "get" ]; then
            echo "Receiving files to /tmp"
            ts file get /tmp
            
            echo "Extracting tar.gz files from /tmp to current directory"
            for tarfile in /tmp/*.tar.gz; do
                if [ -f "$tarfile" ]; then
                    echo "Extracting: $(basename "$tarfile")"
                    tar -xzf "$tarfile" -C .
                    echo "Deleting: $tarfile"
                    rm "$tarfile"
                fi
            done
            echo "Done"
        else
            echo "Usage: ts tar <cp|get>"
            echo "  ts tar cp <file-or-directory> <hostname>:  - Archive and send to host"
            echo "  ts tar get                                 - Receive and extract archives"
            return 1
        fi
    else
        tailscale "${@:1}"
    fi
}



# endregion

# region: ROS -----------------------------------------------------------------
if [[ "$ROS_DISTRO" == "noetic" ]]; then
    alias cbs="catkin build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS=\"-isystem /opt/ros/noetic/include\" && source devel/setup.bash && jq -s 'add' build/*/compile_commands.json > compile_commands.json"
    # alias cbs="catkin build && source devel/setup.bash"
    alias s="source devel/setup.bash"
    alias plotjuggler="rosrun plotjuggler plotjuggler -n"
    export DISABLE_ROS1_EOL_WARNINGS=1
    export CMAKE_POLICY_VERSION_MINIMUM=3.5
elif [[ -n "$ROS_DISTRO" ]]; then
    alias rosbasics="sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp ros-$ROS_DISTRO-foxglove-bridge"
    alias cbs="colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash && jq -s 'add' build/*/compile_commands.json > compile_commands.json"    
    alias s="source install/setup.bash"
    alias plotjuggler="ros2 run plotjuggler plotjuggler -n"
    alias roscore="ros2 run rmw_zenoh_cpp rmw_zenohd"
    alias foxglove="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
    alias foxglove-remote="ros2 launch foxglove_bridge foxglove_bridge_launch.xml use_compression:=true"
    export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
fi

# endregion

# region: VARIABLES -----------------------------------------------------------

export XDG_SESSION_TYPE=x11
export UV_NO_BUILD_ISOLATION=true
export UV_PYTHON_PREFERENCE="system"

# endregion -------------------------------------------------------------------
