#!/bin/bash
filename="Brave-$(date +%s%N)"
cp -r /home/gavin/.config/BraveSoftware/Brave-Browser /tmp/$filename
rm /tmp/$filename/Singleton*
/usr/bin/brave-browser-stable --class=Private --user-data-dir=/tmp/$filename --incognito
rm -rf /tmp/$filename
