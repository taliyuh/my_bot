Followed tutorials by Articulated Robotics on YouTube: https://www.youtube.com/@ArticulatedRobotics.

Run following commands to launch:

ros2 launch my_bot launch_sim.launch.py world:=obstacles.world
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot-main/config/mapper_params_online_async.yaml use_sim_time:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot-main/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
