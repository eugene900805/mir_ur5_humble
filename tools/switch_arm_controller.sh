#!/bin/bash
# Switch the UR5 trajectory controller between:
#   passthrough - whole trajectory executed onboard (immune to WiFi jitter; default)
#   scaled      - 125Hz servoj streaming (tighter monitoring; stutters on WiFi)
# MoveIt knows both (controllers.yaml) and routes to whichever is active,
# so no MoveIt restart is needed. Switch while the arm is idle.
# Usage: switch_arm_controller.sh [passthrough|scaled]   (no arg = show state)
set -e
SCALED=scaled_joint_trajectory_controller
PASST=passthrough_trajectory_controller

case "${1:-}" in
  passthrough)
    ros2 control set_controller_state $SCALED inactive
    ros2 control set_controller_state $PASST active
    ;;
  scaled)
    ros2 control set_controller_state $PASST inactive
    ros2 control set_controller_state $SCALED active
    ;;
  "")
    ;;
  *)
    echo "usage: $0 [passthrough|scaled]"; exit 1
    ;;
esac
ros2 control list_controllers | grep -E "$SCALED|$PASST"
