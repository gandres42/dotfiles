#!/bin/bash
npx asar extract /opt/Rambox/resources/app.asar /tmp/rambox_asar
cd /tmp/rambox_asar/tray
for f in *_light.png *_light.ico *_light@2x.png; do
    dark="${f/_light/_dark}"
    [ -f "$dark" ] && cp "$dark" "$f"
done
cd /tmp
npx asar pack rambox_asar app.asar
sudo cp /tmp/app.asar /opt/Rambox/resources/app.asar