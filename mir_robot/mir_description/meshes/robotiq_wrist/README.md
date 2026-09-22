# Robotiq wrist assembly

The combined MiR100 + UR5 model now mounts:

`ur_tool0 -> FT 300 mounting plate -> FT 300 -> Wrist Camera -> 2F-85`.

The FT 300 output face is 41.5 mm from `ur_tool0`. The camera adds
13.5 mm when used directly with a Robotiq gripper (no camera tool plate).
`robotiq_wrist_camera_tool_link` and `robotiq_85_base_link` are therefore
55 mm along `ur_tool0` +Z, with the same orientation. `ur_tool0` itself
remains the UR flange/tool reference, not the gripper TCP. The RealSense
D435i and its original bracket are also present. `d435i_mount_frame` attaches
beside the gripper at `robotiq_wrist_camera_tool_link`, preserving the original
camera/bracket geometry and its transform relative to the gripper. Relative
to the old flange mounting, both move 55 mm along tool +Z. The inverse
flange-to-tool rotation in this frame preserves the original optical axes.

The old bracket position intersected the FT 300; the new position seats the
bracket on the Wrist Camera output face. Collision checking against the actual
camera CAD mesh reports only about 2.3 micrometres at this mating plane, so that
fixed contact is marked Adjacent in the SRDF. Checks against FT 300, the moving
fingers, the rest of the arm and the chassis remain enabled for the D435i.
This describes a simulation mounting arrangement; verify mounting hardware
and camera extrinsics against the physical robot before using them on hardware.

FT 300 is the assumed sensor variant; this is not an FT 300-S model.
Verify that variant and the physical cable orientation against the actual robot.

## Editing, color and measured length

Edit `urdf/include/robotiq_wrist_stack.xacro` in `mir_description` for the wrist
links, mounting transforms and material. Its `rgba` macro argument defaults to
matte black (`0.025 0.025 0.025 1`). The combined `mir_100_v1.urdf.xacro` connects
the gripper to `robotiq_wrist_camera_tool_link`. Both simulator launch files
expand this same URDF; MoveIt also loads the updated `mir_100.srdf`.

Isaac's STL importer assigns white mesh materials even when URDF specifies a
color. `isaac_sim/convert_to_usd.py` now calls `apply_wrist_materials.py`, which
reads the URDF colors and overrides the imported materials. RViz reads the
URDF colors directly. After editing, rebuild the ROS packages and regenerate
the USD as described in `isaac_sim/README_isaac.md`; restart the running apps.

Axial length from the UR5 mounting face (`ur_tool0`) to the most distant
fingertip surface, measured from this model's collision meshes and mimic-joint
transforms:

| Component / state | Length |
| --- | ---: |
| FT 300 including UR coupling, to output face | 41.50 mm |
| Wrist Camera added mounting height | 13.50 mm |
| 2F-85 open, master joint 0 rad | 149.35 mm |
| 2F-85 closed, master joint 0.7929 rad | 162.87 mm |
| Complete assembly, open | **204.35 mm** |
| Complete assembly, closed | **217.87 mm** |

These are geometric model measurements, not a calibrated grasp TCP or a
measurement of the physical robot. The total varies as the fingers move.
Reproduce after geometry edits with `python3 isaac_sim/measure_wrist_stack.py`
from the repository (requires numpy and trimesh).

The restored D435i/bracket were checked at 17 gripper openings across three
arm poses (51 configurations), allowing only the intended mounting contacts.
MoveIt was validated on the simulated combined robot: collision-aware FK/IK
at `robotiq_85_base_link`, OMPL planning, and trajectory execution for a small
shoulder-pan movement and return to the initial pose all succeeded. The live
arm, gripper and base controllers were active during the test.

## Sources and rights

Retrieved 2026-09-21:

* `robotiq_ft300.stl` and `robotiq_ft300_mounting_plate.stl`:
  [ROS-Industrial Robotiq repository](https://github.com/ros-industrial/robotiq/tree/45196f6558fe8ba9d89bc8a105396c68c3e7e892/robotiq_ft_sensor).
  Original visual mesh paths are `meshes/visual/robotiq_ft300.STL` and
  `meshes/visual/mountings/robotiq_ft300-G-062-COUPLING_G-50-4M6-1D6_20181119.STL`.
  The 41.5 mm offset and sensor mass/inertia come from
  `urdf/robotiq_ft300.urdf.xacro` at the same commit. Mesh coordinates are
  already metres. The BSD notice is retained in `LICENSE.ros-industrial`.
* `robotiq_wrist_camera.stl`: tessellated from Robotiq's
  [official CAD download](https://blog.robotiq.com/hubfs/support-files/WRIST_CAMERA_20171116-Sep-06-2024-02-41-22-7147-PM.step),
  listed under Wrist Camera / Product CAD on the
  [manufacturer's support page](https://robotiq.com/support).
  The CAD is manufacturer-provided material, not covered by the ROS-Industrial
  BSD license. No separate redistribution license was included with the STEP;
  retain this attribution and check manufacturer terms before redistributing it.
* Camera dimensions and nominal optical location:
  [Robotiq Wrist Camera manual, sections 5.1 and 5.1.1](https://blog.robotiq.com/hubfs/support-files/Wrist_Camera_Instruction_Manual_PDF_20210615.pdf).

## CAD conversion

FreeCAD `Part.Shape.read()` was used to read the STEP. The outer housing,
rear housing, two light windows, lens and locating pin (solids 0, 1, 2, 3, 56,
57) were retained; internal electronics were omitted. `MeshPart.meshFromShape`
used `LinearDeflection=0.3`, `AngularDeflection=0.5`, `Relative=False`.
The mounting plane is CAD Z=-1 mm: translate by [0, 0, +1] mm, then scale all
coordinates by 0.001. The result contains 22,028 triangles and spans
75 x 87.5 x 22.4 mm, including the rear camera overhang.

The sensor mesh is rotated by pi about Y at its output face, following the
upstream macro. `robotiq_ft_frame` preserves the upstream measurement-axis
rotation relative to that mesh. The camera's nominal optical frame is at
[0, 35.7, 0.1] mm relative to its mounting plane, tilted 30 degrees toward -Y;
it is not a hand-eye calibration.

## Simulation scope

These additions provide visual/collision geometry, mass/inertia, fixed joints
and TF frames for RViz, MoveIt and the exported Isaac USD. Camera and coupling
inertias are geometric approximations; the coupling mass is estimated at 30 g.
Collision geometry uses conservative cylinders/boxes instead of CAD internals.
The SRDF excludes mating rigid surfaces while retaining checks against other
robot links.

No Robotiq camera image publisher, FT wrench publisher or physical hardware
driver is added. The Isaac `--camera` option targets the restored D435i model
and its existing `/realsense/...` publishers; it does not enable the Robotiq
camera. D435i link/topic names and internal optical transforms are retained.
Rebuild
`mir_description` and `ur_moveit_config` and regenerate the Isaac snapshot after
editing the stack; see `isaac_sim/README_isaac.md` in the repository root.
