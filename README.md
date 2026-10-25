# CS 237A / AA274A section repository

Use `main` for the latest completed work. Start lab 5 on a new `section5` branch,
then push it and merge its pull request into `main` after the lab.

## Repository layout

```text
AA274_Section/
├── autonomy_repo/    # HW1/lab 4 heading controller and HW2/lab 5 navigator
│   ├── scripts/      # heading_controller.py, navigator.py, p3_plot.py
│   ├── launch/       # heading_control.launch.py, navigator.launch.py
│   ├── rviz/
│   ├── plots/
│   ├── CMakeLists.txt
│   └── package.xml
├── s2_basic/         # lab 2 constant-control ROS package
├── s3_basic/         # lab 3 ROS package, launch file, and RViz configuration
├── team.txt
└── README.md
```

Lab 4 brought the HW1 `autonomy_repo` package into the group repository. HW2
problem 4 added `autonomy_repo/scripts/navigator.py`,
`autonomy_repo/launch/navigator.launch.py`, and the executable's CMake install
entry. The required homework algorithms are included in this package; the lab
laptop does not need separate HW1 or HW2 repositories.

The lab 1 exercises and HW1 written report are archived outside the repository.
Their original versions also remain in Git history. Each ROS package's own
`scripts/` directory is required and remains in place.

## Lab 5 setup

On a course laptop with its ROS environment sourced and a fresh workspace:

```bash
mkdir -p ~/autonomy_ws/src
cd ~/autonomy_ws/src
git clone --branch main https://github.com/jackziti/AA274_Section.git
cd AA274_Section
git switch -c section5
cd ~/autonomy_ws
colcon list
colcon build
source install/setup.bash
ros2 pkg executables autonomy_repo
```

Build from `~/autonomy_ws`. The generated `build/`, `install/`, and `log/`
directories belong there, outside this Git repository. `colcon list` should show
`s2_basic`, `s3_basic`, and `autonomy_repo` from this repository.

The course environment must provide `asl_tb3_lib`, `asl_tb3_msgs`, `asl_tb3_sim`,
RViz, NumPy, SciPy, and Matplotlib. The navigator is already integrated, so no
additional homework files need to be copied for lab 5.

## Simulation

First terminal:

```bash
source ~/autonomy_ws/install/setup.bash
unset DISABLE_GUI INSIDE_VM
ros2 launch asl_tb3_sim root.launch.py
```

Second terminal:

```bash
source ~/autonomy_ws/install/setup.bash
ros2 launch autonomy_repo navigator.launch.py
```

Select a goal with **2D Goal Pose** in RViz and demonstrate navigation to the CA.
The navigator launch defaults to `use_sim_time:=true`.

## Hardware

Close the simulator and its navigator before starting the hardware workflow.
Check `echo "$ROS_DOMAIN_ID"` against the assigned laptop/robot domain. Following
the CA's setup, connect to the robot and bring it up:

```bash
ssh aa274@<robot_name>.local
ros2 launch asl_tb3_driver bringup.launch.py
```

In a new **laptop** terminal:

```bash
source ~/autonomy_ws/install/setup.bash
ros2 topic list
ros2 launch autonomy_repo navigator.launch.py use_sim_time:=false
```

Select goals in RViz and demonstrate navigation to the CA.

## Earlier lab nodes

- Lab 2: `ros2 run s2_basic constant_control.py` publishes every 0.2 seconds.
  The original commands are 2.0 m/s and 1.0 rad/s.
- Lab 3: `ros2 launch s3_basic section3.launch.py use_sim_time:=true` starts its
  straight-line controller and RViz. Use `use_sim_time:=false` on hardware.
  The original commands are 2.0 m/s and zero angular velocity.
- Lab 4: `ros2 launch autonomy_repo heading_control.launch.py` defaults to
  simulation time; add `use_sim_time:=false` on hardware. Adjust the gain with
  `ros2 param set /heading_controller kp 3.0`. Run
  `ros2 run autonomy_repo p3_plot.py` from `~/autonomy_ws` to save the plot in
  `src/AA274_Section/autonomy_repo/plots/p3_output.png`.

Run each controller separately. Lab 5 launches the navigator. The lab 2/3
constant-control nodes stop on:

```bash
ros2 topic pub --once /kill std_msgs/msg/Bool '{data: true}'
```

## Validation

Eighteen offline checks passed for package structure, executable installation
paths, Python syntax, launch settings, runtime dependencies, RViz configurations,
heading gain updates, kill messages, planning, trajectory tracking, and plot
output. These included three successive goals through the course navigation
state machine using an ideal unicycle model. Real `colcon list` found all three
packages. ROS transport and launch objects were replaced by test doubles in the
offline checks.

A real ROS build, Gazebo test, and hardware demonstration must still be performed
on the course laptop.
