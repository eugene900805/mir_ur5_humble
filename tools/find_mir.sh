#!/bin/bash
# Find the MiR base on the current network and print the IP to use as
# mir_hostname. Detection order:
#   1. mir.com DNS  — works when connected to the MiR's own hotspot
#      (e.g. "Aislab_mir3"): the robot's DNS resolves mir.com to itself.
#   2. Known candidates — fast check of addresses this lab has used.
#   3. Subnet scan — probe every host on the local /24 for an open
#      rosbridge port (9090). Slowest, but works on any network.
#
# Usage:  ./tools/find_mir.sh
#         ros2 launch mir_navigation mir_nav_launch.py mir_hostname:=$(./tools/find_mir.sh)

has_rosbridge() {  # ip -> 0 if rosbridge (9090) answers
  timeout 1 bash -c "echo > /dev/tcp/$1/9090" 2>/dev/null
}

# 1) mir.com (MiR hotspot DNS)
ip=$(getent hosts mir.com | awk '{print $1}' | head -1)
if [ -n "$ip" ] && has_rosbridge "$ip"; then
  echo "$ip"; exit 0
fi

# 2) known candidates (MiR internal address, TP-Link client address)
for ip in 192.168.12.20 192.168.0.104; do
  if has_rosbridge "$ip"; then echo "$ip"; exit 0; fi
done

# 3) scan the WiFi /24 for port 9090
subnet=$(ip -4 addr show wlo1 2>/dev/null | grep -oE '192\.168\.[0-9]+' | head -1)
if [ -n "$subnet" ]; then
  for i in $(seq 1 254); do
    ( has_rosbridge "$subnet.$i" && echo "$subnet.$i" ) &
  done | head -1
  wait 2>/dev/null
  exit 0
fi

echo "MiR not found (is the robot on and on the same network?)" >&2
exit 1
