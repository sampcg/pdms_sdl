# pdms_gripper_description

Custom flat-fork fingers for the myCobot 320 Pi adaptive gripper.

Overlay package — it ships only the fork meshes and a URDF, and pulls the arm
and the rest of the gripper from `mycobot_description`. Nothing in
`mycobot_ros2` needs editing.

## Build

```bash
ln -s /path/to/PDMS_fabrication/pdms_gripper_description ~/colcon_ws/src/
cd ~/colcon_ws && colcon build --packages-select pdms_gripper_description
source install/setup.bash
```

## Run

```bash
ros2 launch pdms_gripper_description fork_gripper_display.launch.py
```

No hardware needed. Drag the **`gripper_controller`** slider to open and close
the forks.

> Run only one launch at a time. Each `joint_state_publisher_gui` publishes
> `/joint_states` continuously, so two live launches fight and the sliders look
> frozen. `ros2 node list` should show every name exactly once.

## Adjust the finger pose

```bash
ros2 launch pdms_gripper_description fork_gripper_display.launch.py            # terminal 1
python3 $(ros2 pkg prefix --share pdms_gripper_description)/tools/fork_jog.py  # terminal 2
```

Sliders in mm and degrees for both fingers; RViz updates live, no rebuild.
"Copy values" writes the current origins to `~/fork_origin.txt`.

## Notes

- Fork DAEs need `<unit meter="0.001" name="millimeter"/>`. ImageToStl.com
  exports omit it, COLLADA then defaults to 1 m, and the part loads 1000× too
  big. Add it back if you re-export.
- `tools/fit_fork.py` solves the finger pose; `tools/render_urdf.py` renders a
  URDF offscreen. See the header comment in each.
