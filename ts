#!/bin/bash

if [ "$1" == "cloak" ]; then
    tailscale set --exit-node=$(tailscale exit-node suggest | awk -F': ' '/Suggested exit node:/ {print $2}' | sed 's/\.$//')
    ipcheck
elif [ "$1" == "uncloak" ]; then
    tailscale set --exit-node=
    ipcheck
else
    tailscale "${@:1}"
fi
