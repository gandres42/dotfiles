#!/bin/bash
rsync -aP --ignore-existing ./color-schemes/ $HOME/.local/share/color-schemes
cp ./kwinrulesrc ~/.config
qdbus org.kde.KWin /KWin reconfigure
