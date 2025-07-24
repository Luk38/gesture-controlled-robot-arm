# gesture-controlled-robot-arm

## Docker

### Build with:

    docker build -t deoxys docker_deoxys/

### Run with:

    docker run -it -v /cshome:/cshome -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --privileged deoxys

removed after stopping:

    docker run -it --rm -v /cshome:/cshome -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --privileged deoxys

## Panda

### autoscripts

    ./auto_scripts/auto_arm.sh config/charmander.yml

    ./auto_scripts/auto_gripper.sh config/charmander.yml

### reset robot pose

    python examples/reset_robot_joints.py

### Robosuite für Simulation
`pip install robosuite`

### Leap-tracking-service 
`wget -qO - https://repo.ultraleap.com/keys/apt/gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ultraleap.gpg`

`echo 'deb [arch=amd64] https://repo.ultraleap.com/apt stable main' | sudo tee /etc/apt/sources.list.d/ultraleap.list`

`sudo apt update`

`sudo apt install ultraleap-hand-tracking-service`

`(sudo apt install ultraleap-hand-tracking-control-panel)`

`leapctl eula`

### Run in Container
In different terminals

hand tracking programm for the leap motion controller:

    python gesture-controlled-robot-arm/workspace/hand_tracking/hand_positions.py

osc pose control for the robot:

    python gesture-controlled-robot-arm/workspace/robot_control/osc_pose.py 


## Notizen
### Für LeapC import

import sys

sys.path.append("/usr/lib/ultraleap-hand-tracking-service")

### Terminal im Container

docker exec -it 1900ac99c53f /bin/bash
