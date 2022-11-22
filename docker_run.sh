#!/usr/bin/env bash
xhost +local:

    docker run \
    -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -p 2233:22 \
    --rm \
    --name ros \
    --user root \
    --network host \
    -e GRANT_SUDO=yes \
    -v ~/sdc_ws:/root/catkin_ws \
    softmac/sdc-course-docker:midterm \
    bash