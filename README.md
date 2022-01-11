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

    Or multiple goals:
    ```bash
    rostopic pub /list_gps_goals al_manager/list_gps_goals "{gpsgoals: [{latitude: 45.3781861, longitude: -71.9264241}, {latitude: 45.3784642, longitude: -71.9267283}]}" -l
    ```

If the goals arent received by the goal_manager it means the gps_viz app was opened before the autonomous node.