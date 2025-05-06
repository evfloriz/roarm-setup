#!/bin/sh

# Script should be run from the src/ folder ("sh .docker/run.sh")

ROSUSER=ros

docker container run -it --user=$ROSUSER --network=host --ipc=host \
    -v $PWD:/home/$ROSUSER/roarm_ws/src \
    -w /home/$ROSUSER \
    --env=DISPLAY \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    roarm-image
