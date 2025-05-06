#!/bin/sh

# Script is run in /docker folder two level deep from the roarm_ws/ folder

ROSUSER=ros

docker container run -it --user=$ROSUSER --network=host --ipc=host \
    -v $PWD/../..:/home/$ROSUSER/roarm_ws \
    -w /home/$ROSUSER \
    --env=DISPLAY \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    roarm-image
