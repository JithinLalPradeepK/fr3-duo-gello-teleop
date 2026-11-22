# Robot demo host environment. Source me, don't execute:
#   source ~/robot_demo/env/host_env.sh
# Order matters: ROS -> gello_deps_ws -> gello_software/ros2 -> zed_ws.
source /opt/ros/humble/setup.bash
source ~/gello_deps_ws/install/local_setup.bash        # libfranka 0.18.2, franka_ros2, robotiq
source ~/gello_software/ros2/install/local_setup.bash   # GELLO
source ~/zed_ws/install/local_setup.bash               # zed-ros2-wrapper
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ROS 2 traffic on loopback only - NEVER on the robot network (enp97s0 = FCI). See config/cyclonedds_host.xml.
# Tests that set their own CYCLONEDDS_URI / ROS_DOMAIN_ID afterwards still override this.
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/cyclonedds_host.xml"
