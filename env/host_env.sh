# Robot demo host environment. Source me, don't execute:
#   source ~/robot_demo/env/host_env.sh
# Order matters: ROS -> gello_deps_ws -> gello_software/ros2 -> zed_ws.
source /opt/ros/humble/setup.bash
source ~/gello_deps_ws/install/local_setup.bash        # libfranka 0.18.2, franka_ros2, robotiq
source ~/gello_software/ros2/install/local_setup.bash   # GELLO
source ~/zed_ws/install/local_setup.bash               # zed-ros2-wrapper
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
