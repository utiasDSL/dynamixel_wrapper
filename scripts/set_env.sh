export ROS_DOMAIN_ID=101
export GIT_LFS_SKIP_SMUDGE=1
export CRISP_CONFIG_PATH=/home/linusschwarz/repos/crisp_configs
export NETWORK_INTERFACE=enp128s31f6
export ROS_NETWORK_INTERFACE=enp128s31f6
# export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/linusschwarz/repos/crisp_gym/scripts/cyclone_config.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop && ros2 daemon start