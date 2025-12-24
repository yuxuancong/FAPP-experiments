# Copilot Instructions for FAPP

## Project Overview
- **FAPP** (Fast and Adaptive Perception and Planning) is a ROS-based UAV system for real-time obstacle avoidance in dynamic, cluttered environments.
- The codebase is organized as a ROS catkin workspace, with each major component in its own package under `FAPP/`.
- Key components include perception, planning, mapping, control, and simulation, each in a dedicated subdirectory (e.g., `fapp_planner/`, `mot_mapping/`, `so3_control/`).

## Architecture & Data Flow
- **Perception**: Processes sensor data (e.g., point clouds) in real time, typically in `mot_mapping/` and `local_sensing_node/`.
- **Planning**: Core planning logic is in `fapp_planner/` and `planner/` (see `src/planner/plan_manage/src/ego_replan_fsm.cpp`).
- **Control**: Low-level control and simulation in `so3_control/` and `so3_quadrotor_simulator/`.
- **Communication**: Uses ROS topics/services for inter-component messaging. Message definitions are in `quadrotor_msgs/` and `obj_state_msgs/`.

## Developer Workflows
- **Build**: Use `catkin build` from the `FAPP` root. Ensure ROS Noetic and dependencies are installed.
- **Launch**: Use `tmuxp load quick_start.yaml` or launch nodes individually as described in the README.
- **Testing**: No explicit test suite; validate by running simulation and checking Rviz outputs.
- **Debugging**: Use ROS tools (`rosnode`, `rostopic`, `rqt_graph`) and Rviz for visualization.

## Project Conventions
- **C++** is the primary language for core logic; follow ROS C++ conventions.
- **Python** scripts may be present for utilities or launch management.
- **All ROS nodes** are launched/configured via YAML or launch files in the root or package directories.
- **Message passing** is strictly via ROS topics/services; avoid direct cross-package calls.
- **Simulation** and real-world code are kept together; check for `sim` or `test` folders for scenario-specific logic.

## Integration & External Dependencies
- Depends on ROS Noetic, tmux, and tmuxp.
- Integrates with [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) and [EGO-Planner](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) for trajectory optimization and planning.
- See `quick_start.yaml` for the canonical launch sequence and node dependencies.

## Key Files & Directories
- `src/planner/plan_manage/src/ego_replan_fsm.cpp`: Main planning FSM logic.
- `quick_start.yaml`: Launch configuration for the full system.
- `fapp_planner/`, `mot_mapping/`, `so3_control/`: Core ROS packages.
- `quadrotor_msgs/`, `obj_state_msgs/`: Custom ROS message definitions.
- `README.md`: High-level documentation and usage instructions.

## Example Patterns
- ROS node initialization and spinning in each `src/` directory.
- Parameters loaded from YAML or via ROS parameter server.
- Data flows: Sensor → Perception → Mapping → Planning → Control → Actuation.

---
For more details, see the [README.md](../README.md) and comments in key source files.
