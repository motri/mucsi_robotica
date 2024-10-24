#!/bin/bash -e

sudo -u laboratorio mkdir -pm700 /tmp/runtime-laboratorio
sudo chown laboratorio:laboratorio /tmp/runtime-laboratorio
sudo -u laboratorio chmod 700 /tmp/runtime-laboratorio

sudo ln -snf "/usr/share/zoneinfo/$TZ" /etc/localtime && echo "$TZ" | sudo tee /etc/timezone > /dev/null

sudo /etc/init.d/dbus start

Xvfb "${DISPLAY}" -ac -screen "0" "${RESOLUTION}" -dpi "${DPI}" +extension "RANDR" +extension "GLX" +iglx +extension "MIT-SHM" +render -nolisten "tcp" -noreset -shmem &

echo "Waiting for X socket"
until [ -S "/tmp/.X11-unix/X${DISPLAY/:/}" ]; do sleep 1; done
echo "X socket is ready"

x11vnc -display "${DISPLAY}" -passwd laboratorio -shared -forever -repeat -xkb -snapfb -threads -xrandr "resize" -rfbport $VNC_PORT &
/opt/noVNC/utils/novnc_proxy --vnc localhost:$VNC_PORT --listen $NOVNC_PORT --heartbeat 10 &

startplasma-x11