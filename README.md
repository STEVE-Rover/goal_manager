# goal_manager

Set a ROS navigation goal using a gps coordinate or multiple gps coordinate.

## Usage
1. launch the goal_manager node :
    ```bash
    roslaunch goal_manager goal_manager.launch   
    ```

2. Send a goal:
    ```bash
    rostopic pub /gps_goal_fix sensor_msgs/NavSatFix "{latitude: 38.42, longitude: -110.79}" -1
    ```
