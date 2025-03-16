#!/bin/bash

if [ "$1" == "mull" ]; then
    tailscale set --exit-node=$(tailscale exit-node suggest | awk -F': ' '/Suggested exit node:/ {print $2}' | sed 's/\.$//')
    ipcheck
elif [ "$1" == "unmull" ]; then
    tailscale set --exit-node=
    ipcheck
else
    tailscale "${@:1}"
fi
