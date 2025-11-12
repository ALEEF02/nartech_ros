# nartech_ros

<img width="1071" alt="Image" src="https://github.com/user-attachments/assets/6fcfeaab-55b6-4637-900d-85dc41922a97" />

### Run all necessary ROS nodes, Gazebo simulation, and AI demo file from demos folder:

```./start_nartech.bash metta demo_with_nars.metta```

This will:
1. Open a Gazebo sim instance showing the `narhouse.sdf` world (Tab #2)
2. Open `demo_with_nars.metta` in Geany
3. Execute `/home/nartech/nartech_ws/src/nartech_ros/main.py` on `demo_with_nars.metta` (Tab #3)
4. Launch ros2 [nav2_bringup](https://index.ros.org/p/nav2_bringup/) on `/home/nartech/nartech_ws/src/nartech_ros/manipulation/nav2_bringup/launch/tb4_simulation_launch.py`
   - This is the TurtleBot4 standardized(?) bringup file
   - Is a headless instance using SLAM & RVIZ (`nartech_view.rviz`)
5. Run ros2 controller_manager gripper_controller
6. Launch MoveIt (tb4_openmanipulator_moveit)
7. Continues running to clear the MoveIt Octomap every 2 seconds

To shut it down:
1. Ctrl+C in the `./start_nartech.bash metta demo_with_nars.metta` terminal window (#1)
2. `pkill -f "python3 .*nartech_ros/main.py .*\.metta"`
3. `ros2 run controller_manager unspawner gripper_controller --controller-manager /controller_manager`
4. `pkill -2 -f "ros2 launch .*move_group.launch.py"`
5. `pkill -2 -f "ros2 launch .*tb4_simulation_launch.py"`
6. `pkill -2 -f "gz sim --render-engine ogre"`

### For developers, run all necessary ROS nodes, Gazebo simulation, and MeTTa interface first:

```./start_nartech.bash metta```

then open Geany or another Terminal to run the particular AI demo:

```cd /home/nartech/nartech_ws/src/nartech_ros/```

```python3 main.py ./demos/demo_with_nars.metta```

Benefit of this is that this Python3 script can be safely stopped (```sudo killall -9 python3```) and restarted without affecting crucial ROS components and the Gazebo simulation,
this allows continuous work on AI demos in MeTTa.

### Run monolithic NACE+NARS+GPT input demo:

```./start_nartech.bash```

### Test various MeTTabridge plugins without running ROS:

```python3 mettabridge.py ./plugins/Xplugintests/file.metta```

These plugins can also be used standalone as MeTTa extensions via the MeTTa import command.
