#!/bin/bash
# Maze narrow-corridor stress test: N south<->north round trips, each of which
# must thread the 0.933 m gap. Prints a per-lap result you can feed to
# summarise.py. Judge any base or Nav2 parameter change on a run of this, never
# on a single lap -- with 17.6 cm of clearance per side the lap-to-lap spread is
# larger than most of the effects being tuned.
#
# Prerequisites (each in its own terminal, see README_isaac.md §6):
#   1. mir_isaac_sim.py --headless --lasers --world isaac_sim/usd/maze.usd
#   2. ros2 launch mir_description mir_isaac.launch.py launch_moveit:=false launch_rviz:=false
#   3. ros2 launch mir_navigation amcl.py use_sim_time:=true map:=.../maze.yaml
#   4. ros2 launch mir_navigation navigation.py use_sim_time:=true \
#          cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
#   ...then seed AMCL once (RViz 2D Pose Estimate, or publish /initialpose).
#
# Usage: ./stress.sh [laps]        (default 5 laps = 10 goals)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N=${1:-5}
for i in $(seq 1 "$N"); do
  echo "##### trial $i N->S"; python3 "$HERE/maze_run.py" 1.0 0.0 3.1416 200 2>&1 | tail -8
  echo "##### trial $i S->N"; python3 "$HERE/maze_run.py" 5.5 5.0 1.5708 200 2>&1 | tail -8
done
