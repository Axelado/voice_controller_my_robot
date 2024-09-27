# voice_controller_my_robot

## Description
`voice_controller_my_robot` is a ROS 2 package that enables voice commands to control a robot's navigation. The current implementation allows users to issue commands to direct the robot to specific rooms by voice. Each room is associated with a number, and its corresponding coordinates are specified in the `data/rooms_data.json` file. The package leverages voice recognition to interpret commands and translates them into navigation goals within a pre-defined map.

## Features
- **Voice-controlled navigation**: The robot can navigate to rooms by processing spoken commands.
- **Room mapping**: Rooms are associated with their coordinates in the map via a JSON configuration file.
- **Simulation support**: Can be used in a simulated environment with simulated time.

## Files and Configuration
- **rooms_data.json**: This file contains the mapping between room numbers and their corresponding coordinates on the map. It is located in the `data/` directory.
  
  Example structure of `rooms_data.json`:
  ```json
  {
    "1": {"x": 1.5, "y": 3.2},
    "2": {"x": 2.3, "y": 4.1},
    "3": {"x": 3.0, "y": 5.6}
  }
  ```

## Launch File Overview
The `voice_controller.launch.py` launch file is used to start the necessary nodes for voice control.

### Nodes:
- **keyboard_activator**: This node allows for the manual activation of voice commands through the keyboard.
- **voice_recorder**: This node handles the voice recording and recognition functionality, converting spoken commands into text.
- **pose_publish_from_room_number**: Based on the recognized room number, this node publishes the corresponding goal pose for the robot to navigate to.

### Launch Arguments:
- **use_sim_time**: 
  - Default: `true`
  - Enables or disables the use of simulation time (especially useful when testing in simulated environments like Gazebo).

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository_link>
   cd voice_controller_my_robot
   ```

2. **Install dependencies**:
   Ensure you have installed the necessary dependencies for ROS 2 and any additional packages required for voice recognition.

3. **Build the workspace**:
   Build the package within your ROS 2 workspace:
   ```bash
   colcon build
   ```

4. **Source the workspace**:
   Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage

To start the voice control system, use the following command:
```bash
ros2 launch voice_controller_my_robot voice_controller.launch.py
```

### Example Workflow:
1. **Activate voice command**: Use the keyboard or any trigger mechanism to activate the voice recognition system.
2. **Issue voice commands**: Speak the number of the room you want the robot to go to (e.g., "Go to room 2").
3. **Navigation**: The robot will use the room coordinates from `rooms_data.json` to navigate to the specified location on the map.

### Customization:
- To modify the room mapping, edit the `data/rooms_data.json` file with the desired room numbers and their corresponding coordinates.
- Adjust the parameters in the launch file to suit your specific environment, such as toggling the use of simulated time.

## Dependencies
- [ROS 2](https://docs.ros.org/en/rolling/Installation.html)
- Voice recognition libraries (Google Speech-to-Text)
- A microphone or audio input device for voice commands

## License
This project is licensed under the [MIT License](LICENSE).

## Contributors
- **Axel NIATO** - Lead Developer