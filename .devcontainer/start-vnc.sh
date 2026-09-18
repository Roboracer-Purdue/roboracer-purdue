#!/bin/bash
# Virtual desktop with OpenGL for RViz — view at http://localhost:6080/vnc.html (password: vscode)

# Already running? Do nothing.
pgrep -x Xvfb >/dev/null && exit 0

export DISPLAY=:99

# Display server with GLX (OpenGL) enabled, 24-bit color
setsid Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
sleep 2

# Desktop environment: XFCE if installed, otherwise Fluxbox
if command -v startxfce4 >/dev/null; then
    setsid dbus-launch startxfce4 >/tmp/desktop.log 2>&1 &
else
    setsid fluxbox >/tmp/desktop.log 2>&1 &
fi

# VNC server with password "vscode"
mkdir -p ~/.vnc
x11vnc -storepasswd vscode ~/.vnc/passwd >/dev/null 2>&1
setsid x11vnc -display :99 -forever -shared -rfbauth ~/.vnc/passwd -rfbport 5900 -quiet >/tmp/x11vnc.log 2>&1 &

# Browser access on port 6080
setsid websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &