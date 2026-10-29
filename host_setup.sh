#!/usr/bin/env bash
set -euo pipefail

# 1. Install HWE kernel and firmware (if not already installed)
sudo apt-get update
sudo apt-get install -y linux-generic-hwe-22.04 linux-firmware

# 2. Blacklist btmtk to prevent Bluetooth coexistence latency spikes
if ! grep -q "blacklist btmtk" /etc/modprobe.d/blacklist-btmtk.conf 2>/dev/null; then
    echo "blacklist btmtk" | sudo tee /etc/modprobe.d/blacklist-btmtk.conf > /dev/null
    sudo modprobe -r btmtk 2>/dev/null || true
fi

# 3. Disable Wi-Fi power management on the wireless interface connected to the drone (192.168.10.1)
WIFI_IF=$(ip route get 192.168.10.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}' | head -n 1)
if [ -z "$WIFI_IF" ]; then
    WIFI_IF=$(iw dev | awk '$1=="Interface"{print $2}' | head -n 1)
fi
if [ -n "$WIFI_IF" ]; then
    sudo iw dev "$WIFI_IF" set power_save off
fi

# 4. Enable Docker X11 passthrough
if command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    xhost +local:docker
fi
