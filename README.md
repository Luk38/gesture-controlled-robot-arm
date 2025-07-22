# gesture-controlled-robot-arm
### Auführen in workspace

python hand_tracking/hand_positions.py

python robot_control/osc_pose.py 

Über Socket

robot_control/get_hand_positions.py

Über getter Funktion

### Für LeapC import

import sys

sys.path.append("/usr/lib/ultraleap-hand-tracking-service")

Unter Windows :C:\Program Files\Ultraleap\LeapSDK\lib\x64

## Docker

### Terminal im Container

docker exec -it 1900ac99c53f /bin/bash

### Build with:

docker build -t deoxys docker_deoxys/

### Run with:

docker run -it -v /cshome:/cshome -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --privileged deoxys

docker run -it --rm -v /cshome:/cshome -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --privileged deoxys

## Panda

### autoscripts

./auto_scripts/auto_arm.sh config/charmander.yml

./auto_scripts/auto_gripper.sh config/charmander.yml

### example

python examples/reset_robot_joints.py

### Notizen
Position (von meiner Blickrichtung):
Panda x Richtung nach unten,
y nach rechts, z nach hinten

Rotation: