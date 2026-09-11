#!/bin/bash
set -e

# Source ROS 2 Humble environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi

# Source workspace environment if built
if [ -f "/aiformula_ws/install/setup.bash" ]; then
    source "/aiformula_ws/install/setup.bash"
fi

# Set virtual display
export DISPLAY="${DISPLAY:-:1}"

# Ensure X11 / GUI services are running
start_gui_services() {
    # If Xvfb is not running, cleanup stale locks and start it
    if ! pgrep -x "Xvfb" > /dev/null; then
        rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true
        Xvfb :1 -screen 0 1920x1080x24+32 > /tmp/xvfb.log 2>&1 &
        sleep 1
    fi

    # Start Window Manager if not running
    if ! pgrep -x "fluxbox" > /dev/null; then
        fluxbox > /tmp/fluxbox.log 2>&1 &
        sleep 0.5
    fi

    # Start x11vnc if not running
    if ! pgrep -x "x11vnc" > /dev/null; then
        x11vnc -display :1 -forever -shared -nopw -rfbport 5900 -quiet > /tmp/x11vnc.log 2>&1 &
        sleep 0.5
    fi

    # Start websockify (noVNC web server on port 8080) if not running
    if ! pgrep -f "websockify.*8080" > /dev/null; then
        websockify --web /usr/share/novnc 8080 localhost:5900 > /tmp/novnc.log 2>&1 &
        sleep 0.5
    fi
}

start_gui_services

exec "$@"
