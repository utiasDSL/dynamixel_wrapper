export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_NETWORK_INTERFACE="enx607d0937fb24"


SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

export CYCLONEDDS_URI=file:///$SCRIPT_DIR/cyclone_config.xml

ros2 daemon stop && ros2 daemon start
