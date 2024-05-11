xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=halcon_container\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/home/$USER/catkin_ws/src/halcon_package:/catkin_ws/src/halcon_package" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    ros-noetic-halcon \
    bash

echo "Done."