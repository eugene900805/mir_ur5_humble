#!/bin/bash
# Auto-resend External Control when the reverse connection drops.
# The host reaches the UR5 over WiFi; outages longer than the robot-side
# watchdog kill the External Control script. In headless mode it can be
# resent via service call — this loop does that automatically.
# Usage: source install/setup.bash && ./mir_ur5/ur_program_watchdog.sh
ROBOT_IP=${ROBOT_IP:-192.168.0.101}
INTERVAL=${INTERVAL:-5}

running() {
  python3 - "$ROBOT_IP" <<'EOF'
import socket, sys, time
try:
    s = socket.create_connection((sys.argv[1], 29999), timeout=3)
    s.recv(4096)
    s.send(b"running\n"); time.sleep(0.3)
    print("true" if "true" in s.recv(4096).decode() else "false")
    s.close()
except Exception:
    print("unreachable")
EOF
}

while true; do
  state=$(running)
  if [ "$state" = "false" ]; then
    echo "$(date +%H:%M:%S) program not running -> resending External Control"
    timeout 15 ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger >/dev/null 2>&1
    sleep 8   # script takes ~6s to come up; don't hammer
  fi
  sleep "$INTERVAL"
done
