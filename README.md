# gesture-controlled-robot-arm
control schemes found in /workspace

## Docker

### Build with:

    docker build -t deoxys docker_deoxys/

### Run with:

    docker run -it -v /cshome:/cshome -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --privileged deoxys

removed after stopping:

    docker run -it --rm -v /cshome:/cshome -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --privileged deoxys

install Leap-tracking-service in container

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

second control scheme:

    python gesture-controlled-robot-arm/workspace/robot_control/osc_with_linear_velocity.py


## Notizen
### For LeapC import

import sys

sys.path.append("/usr/lib/ultraleap-hand-tracking-service")

### Terminal in Container

docker exec -it id /bin/bash

### Deoxys with Spacemouse

python examples/run_deoxys_with_space_mouse.py --vendor-id id --product-id id
python gesture-controlled-robot-arm/other_controls/run_deoxys_with_space_mouse.py --vendor-id id --product-id id


