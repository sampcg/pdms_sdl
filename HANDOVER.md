# Handover — state of the workcell simulation

## Run it

**Smooth demo (works, use this):**
```bash
source /home/samuel/colcon_ws/install/setup.bash && ros2 launch pdms_gripper_description workcell.launch.py sequence:=true
```
Single holder, single syringe. Timed pick-and-place, 16.5 s cycle. No collision
checking — it interpolates a spline and publishes joint states directly.

**Rail scene, 3 stations (2 syringes + mixer), manual sliders:**
```bash
source /home/samuel/colcon_ws/install/setup.bash && ros2 launch pdms_gripper_description workcell.launch.py model:=$(ros2 pkg prefix --share pdms_gripper_description)/urdf/mycobot_320_pi_2022_workcell_rail.urdf objects:=objects_rail.yaml
```
`objects:=objects_rail.yaml` is required. Without it you get the holders but no
syringes and no mixer: the manifest is opt-in, because loading it against the
single-station URDF parents the markers to sockets that do not exist.
Grasp an object: `ros2 topic pub --once /pipette/grasp std_msgs/Int32 "{data: 0}"`
(0 = left syringe, 1 = right syringe, 2 = mixer, -1 = release)

**Rail scene under MoveIt, collision-checked (fully working):**
```bash
source /home/samuel/colcon_ws/install/setup.bash && ros2 launch pdms_320_moveit pick_place.launch.py station:=0
```

**Only ever run ONE launch at a time.** Two `joint_state_publisher`s or two
`robot_state_publisher`s fight over topics and nothing moves. Check with
`ros2 node list` — every name should appear exactly once. To clear:
```bash
pkill -9 -f rviz; pkill -9 -f robot_state_publisher; pkill -9 -f move_group; pkill -9 -f pipette_attach; pkill -9 -f ros2_control_node
```

## No open bugs

An earlier version of this file reported a gripper mimic-joint bug. **That bug
did not exist** - the diagnostic was wrong, not the robot.

The check parsed `ros2 topic echo /joint_states` with `x.strip('- \n')`, which
strips the leading YAML list dash *and the minus sign*. Every negative joint
value was reported as positive, so `gripper_controller` at its normal `-0.65`
looked like `+0.65` and therefore "outside its `-1.11..0` limit".

Verified properly by subscribing to `/joint_states` with rclpy:

```
gripper_controller                 -0.6618   in range
gripper_base_to_gripper_left2      -0.6618   x +1.0   OK
gripper_left3_to_gripper_left1     +0.6618   x -1.0   OK
gripper_base_to_gripper_right3     +0.6618   x -1.0   OK
gripper_base_to_gripper_right2     +0.6618   x -1.0   OK
gripper_right3_to_gripper_right1   -0.6618   x +1.0   OK
```

`mock_components/GenericSystem` honours `<param name="mimic">` and
`<param name="multiplier">` correctly. The MoveIt path is fully working.

**Lesson:** never parse `ros2 topic echo` output with string stripping. Subscribe
with rclpy, or use `ros2 topic echo --field`.

## What works and is verified

| thing | status |
|---|---|
| fork gripper fitted to the 320 | correct, derived not guessed |
| `TOOL_OFFSET` = 0.1044 | barrel sits between the fork faces |
| `GRIPPER_CLOSED` = -0.78 | actually contacts a 35 mm barrel (-0.55 never did) |
| head-on grasp azimuth | 0 deg at grasp/lift/via, 1.1 mm IK residual |
| spline playback + Cartesian approach/insert | no stop-start, straight-line insertion |
| pipette returns to holder before release | yes |
| grasp offset captured from TF at grasp time | no jump, self-adjusting |
| 3-station rail with 100.6 mm mixer boss | all stations on one 410 mm arc |
| SRDF collision matrix for the rail scene | 78 enabled pairs, 32 arm-vs-workcell |
| gripper cross-side pairs checked | yes, incl. `left1 <-> right1` |
| all 3 grasp targets inside the workspace | 64-80 mm margin |

## Numbers you may need

```
robot base            world origin; arm's front is -Y
head-on reach limit   0.490 m at z=0.217, 0.495 m at z=0.260 and z=0.310
frame origin          (0.26972, -0.56513, 0)
station bores         L (-0.115, -0.410, 0.180)
                      mixer (0.000, -0.410, 0.180)
                      R (+0.115, -0.410, 0.180)
grasp heights         syringe z=0.260 (or 0.310 at its CoM)
                      mixer   z=0.217
syringe CoM           z=0.310, i.e. 190 mm above the nozzle tip
mixer CoM             z=133.5 in its own mesh - INSIDE the holder's grip band,
                      so the arm must grip 30 mm above it
station pitch         115 mm (minimum 102 mm: 140 mm gripper envelope at transit)
gripper travel        -0.05 open / -0.65 transit / -0.78 closed on 35 mm
```

## Gotchas that cost time

1. **`colcon build` must be run from `/home/samuel/colcon_ws`.** If the shell's
   cwd is inside the package, colcon silently creates a throwaway workspace there
   and the real install stays stale. If a change seems to have no effect, this is
   the first thing to check. `rm -rf` any `build/ install/ log/` that appear
   inside `pdms_gripper_description/`.
2. **Every ImageToStl.com export omits `<unit meter="0.001">`**, so meshes load
   1000x oversized. `build_workcell.py` handles collision scaling, but the unit
   tag is added by hand on import. Fusion 360 can't export COLLADA — consider
   exporting STL and using `scale="0.001 0.001 0.001"` in the URDF mesh tag
   instead, which removes this whole class of bug.
3. **`pkill -f <pattern>` can kill the shell running it** if the pattern appears
   in its own command line.

## Regenerating things

```bash
# URDFs (single station, and the 3-station rail)
python3 ~/PDMS_clean/pdms_gripper_description/tools/build_workcell.py
python3 ~/PDMS_clean/pdms_gripper_description/tools/build_workcell.py --rail

# the 3-station rail mesh
python3 ~/PDMS_clean/pdms_gripper_description/tools/make_station_rail.py

# SRDF collision matrix (~3 min; re-run after ANY link is added or renamed)
cd ~/PDMS_clean/pdms_320_moveit && python3 tools/gen_srdf.py

# then always
cd ~/colcon_ws && colcon build --packages-select pdms_gripper_description pdms_320_moveit --cmake-target install
```

## Design work not yet built

From the geometry analysis, still to do in CAD:

- **Horizontal-extraction clip holder.** Design study and a working mesh are
  parked at `~/clip_holder_study/` (outside the project). Buys ~100 mm of reach
  margin because extraction pulls the tool *towards* the robot instead of away.
  Step-by-step Fusion instructions were given in chat.
- **Deepen the pipette bore** 8 mm -> 30 mm. Current depth allows 14 deg of tilt.
- **Chamfer the bore mouth** (45 deg x 3 mm). Insertion currently needs better
  than 1 mm accuracy; the arm's is 1.1 mm.
- **Grip pads on the syringe**, 38-40 mm across flats, at the CoM (190 mm above
  the nozzle), with a locating shoulder below. A printed collar avoids modifying
  a commercial syringe.
- **Grip pads on the mixer** upper band (z 149..178), 40 mm across flats, so one
  gripper value works for both objects instead of -0.78 / -0.82.
- **Anti-rotation key** — the bore and shaft are both round, so the syringe spins
  freely and flats may not face the forks.
