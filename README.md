# suicide_drone_swarm
Suicide swarm drone project using Gazebo classic, ROS2, and PX4.

# 사용 명령어
## Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888

## PX4Swarm
cd ~/PX4Swarm/Tools/simulation/gazebo-classic

./sitl_multiple_run.sh -m iris -n 5(Number of drone) -w ../../../../../../suicide_drone_swarm/src/simulation/worlds/pure

## ROS2 scripts
python3 ~/suicide_drone_swarm/scripts/hover_test.py

python3 ~/suicide_drone_swarm/scripts/balloon_strike.py 

python3 ~/suicide_drone_swarm/scripts/dual_strike.py

python3 ~/suicide_drone_swarm/scripts/swarm_strike.py

python3 ~/suicide_drone_swarm/scripts/swarm_strike_sequential.py


