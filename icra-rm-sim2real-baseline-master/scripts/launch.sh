#!/bin/bash
SERVER_IMAGE=${SERVER_IMAGE:-tb5zhh/icra-2023-server:latest}
# CLIENT_IMAGE=${CLIENT_IMAGE:-tb5zhh/icra-2023-client:latest}
CLIENT_IMAGE=${CLIENT_IMAGE:-docker.discover-lab.com:55555/sztu/client:attack-0.0.2-ai}
CLI_EXE=$@

xhost +

# docker pull $SERVER_IMAGE

docker network create net-sim

docker run -dit --rm --name ros-master --network net-sim ros:noetic-ros-core-focal roscore

docker run -dit --rm --name sim-server --network net-sim \
	--gpus all \
	-e ROS_MASTER_URI=http://ros-master:11311 \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e NO_AT_BRIDGE=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	$SERVER_IMAGE 

sleep 12
docker exec -dit sim-server /opt/ros/noetic/env.sh /opt/workspace/devel/env.sh rostopic pub -1 /reset geometry_msgs/Point 0.0 0.0 0.0
sleep 4
docker run -it --name client --network net-sim \
	--cpus=5.6 -m 8192M \
	-e ROS_MASTER_URI=http://ros-master:11311 \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e NO_AT_BRIDGE=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	$CLIENT_IMAGE $CLI_EXE

# sleep 2
# docker network connect net-sim client 
# docker restart client 
./