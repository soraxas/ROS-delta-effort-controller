# Delta Effort Controller

The `JointEffortDeltaController` is a hybrid between position and effort controller. It accepts both joint positions and effort command, and the effort is applied as a delta to the previous effort from its positional command (to withstand external forces such as gravity).

## Details

It functions as follows:
- It starts as in the position-hold mode, where it will try to maintain the current joint position while withstanding gravity (internally it applies PID control with effort).
- When a delta command is send to the `delta_effort_command` endpoint, it will applies the given effort as a delta to add on top of the current effort.
  - The delta effort is applied for a configurable amount of duration (default: 100 ms).
  - After the fixed duration, it will switch back to position-hold mode, where it will try to hold the latest position.
- Whenever a new position command is sent to the controller, it will always override any active delta effort command, and switch it back to position-hold mode.

## ROS topic endpoints

Normally under the joint name space, e.g. `/j2n6s300/joint_1_delta_effort_controller/...`

- Joint position
  - type: `std_msgs/Float64`
  - desc: The target joint position to hold.
  - `/j2n6s300/joint_1_delta_effort_controller/position_command`
- Delta effort
  - type: `std_msgs/Float64`
  - desc: The amount of delta effort to apply to this joint.
  - `/j2n6s300/joint_1_delta_effort_controller/delta_effort_command`
- Delta effort duration
  - type: `std_msgs/Duration`
  - desc: The duration to apply the delta effort, after which this delta effort will expires and revert back to position-hold mode. When this value is <= 0, it denotes to the controller that this control should be held forever (i.e. do not expires it).
  - `/j2n6s300/joint_1_delta_effort_controller/delta_effort_duration`
