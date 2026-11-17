# Cutebot swarm simulation and data requirements

## What the simulation does

The simulation models **three Cutebots pushing three assigned objects into a shared circular goal**. At each 140 ms step, it tracks each robot’s center and heading, each object’s center, and which object belongs to each robot. The controller converts that scene into 25 numeric features per robot, including relative positions, bearings, and how well the robot is lined up behind its object. All three robots use the same trained neural network. The controller applies turn and stall recovery, then the simulator advances the robots and objects using its 2D motion and contact model. An episode succeeds when all three object centers are inside the goal.

The latest policy learned from a geometric teacher through **behavior cloning and DAgger**. The teacher demonstrates how to go around an object, line up behind it, and push toward the goal. DAgger adds teacher corrections for situations the student encounters while driving. The earlier v7 notebook also tested PPO, which used simulated rewards for object progress and delivery. The latest variable-grid checkpoint used in the replay was trained without PPO. [Variable-grid controller](C:/Users/angry/Downloads/Cutebot_Project/Cutebot_Project/cutebot_variable_grid_controller.zip) · [v7 training notebook](C:/Users/angry/Downloads/Cutebot_Project/Cutebot_Project/cutebot_swarm_imitation_ppo_v7.ipynb)

On the physical setup, **phone tracking replaces the simulator as the source of the scene**. The laptop runs the saved policy and sends commands to the bots. Training and live operation use the same observation encoder.

## What the computer vision group must provide

The laptop controller needs **one complete snapshot from the same camera frame**. The types below describe the Python controller interface. The vision and communications groups can agree on a compact encoding for transmission.

| Field | Data type and shape | Meaning |
|---|---|---|
| `robot_xy_m` | `float32[3, 2]` | `[x, y]` center of each robot in meters, in robot ID order `0, 1, 2`. |
| `heading_rad` | `float32[3]` | Forward heading of each robot, in the same order. Zero points along positive `x`. Angles increase counterclockwise. |
| `object_xy_m` | `float32[3, 2]` | `[x, y]` center of each pushable object in meters, in object ID order `0, 1, 2`. |
| `captured_at` | `float64` seconds | Capture time expressed on the laptop’s monotonic time base. It must advance for each new frame. |
| `host_now` | `float64` seconds | Current time on that same clock, supplied by the laptop adapter when it calls the controller. |

Use one calibrated floor coordinate system, with `(0, 0)` at the agreed arena corner and the positive axes along the marked edges. Convert image coordinates to floor meters. If the phone tracks front and rear markers, transform both marker positions into floor coordinates before computing heading. Keep robot and object IDs stable across frames. If a pose is missing, keep its slot and have the laptop adapter represent the missing measurement as `NaN`. The controller stops all three bots on an invalid snapshot, a repeated capture timestamp, or a snapshot more than 280 ms old. A frame sequence number on the phone link can help detect duplicates, and the laptop needs a way to account for capture age separately from receipt time. [Project interface guide](C:/Users/angry/Downloads/Cutebot_Project/Cutebot_Project/README.md)

**Current gap:** The hub’s existing phone packet carries each robot’s front and rear marker coordinates plus **one target point**. The controller needs centers for **three pushable objects**. The goal is a separate piece of arena configuration. The vision and communications groups need to update the phone payload and hub decoder together. [Laptop hub](C:/Users/angry/Downloads/files/laptop_hub.py)

## What the arena and hardware groups must provide

| Setup field | Data type | Meaning |
|---|---|---|
| `grid_ft` | Pair of floats | Marked arena width and height in feet. Each axis must be at least 4 ft. |
| `goal_center_m` | Pair of floats | Goal center in the same floor coordinates as the tracked positions. |
| `goal_radius_m` | Float | Goal radius in meters, from `0.180` to `0.235`. |
| `camera_min_m`, `camera_max_m` | Two pairs of floats | Lower and upper bounds of usable tracking coverage, in floor meters. The bounds must cover the arena and extend at least 0.43 m beyond the goal boundary. |
| `assignments` | Three distinct integer IDs | Object ID assigned to each robot, in robot ID order. `[2, 0, 1]` means robot 0 pushes object 2. |

The hardware group should provide measured robot and object dimensions and verify how the bots move under each motor action. The simulator uses a **7.5 cm robot radius, a 5 cm object radius, a nominal forward speed of 0.20 m/s, and a nominal turn rate of 3 rad/s**. Comparing real motion and contact behavior with those assumptions helps determine whether motor calibration is sufficient or the simulator needs updating. [Variable-grid controller](C:/Users/angry/Downloads/Cutebot_Project/Cutebot_Project/cutebot_variable_grid_controller.zip)

## What the communications and firmware groups must implement

The laptop needs a stable mapping from each camera robot ID to a specific micro:bit. The firmware needs a command message carrying an **integer sequence number**, an **integer action ID**, and a **tracking-valid flag** or an explicit STOP. The action IDs are `0` stop, `1` forward, `2` rotate left, `3` rotate right, `4` forward-left, `5` forward-right, and `6` reverse. The firmware should reject out-of-order commands and stop on invalid tracking, disconnect, or command expiry. The current controller emits a 140 ms timeout, so verify the command cadence and expiry behavior with all three BLE links active. The existing firmware only responds to a test ping, and the current hub limits each command line to 19 ASCII characters before the newline. [Laptop hub](C:/Users/angry/Downloads/files/laptop_hub.py) · [Micro:bit firmware](C:/Users/angry/Downloads/files/microbit_comms.ts)

**Where the location data goes matters.** In the current architecture, the **laptop policy consumes all robot and object coordinates**, and each Cutebot can execute its assigned action using only the command message. If each physical Cutebot must also *receive and store* location data, specify a separate pose message containing a frame sequence, robot IDs with `(x, y, heading)`, and object IDs with `(x, y)`. Agree on units, numeric precision, and the encoding with the firmware group. The existing command dictionaries include peer robot poses but omit object positions, and neither the hub nor the firmware implements pose delivery today.

For the first integration test, track one robot and one object against measured floor positions, verify heading and BLE device identity, and confirm the robot stops when command updates cease. Then verify that a complete three-robot, three-object frame reaches `CameraHost.dispatch()` and produces three correctly routed commands before enabling autonomous pushing.
