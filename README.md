# Docker

alias kinova_noetic_gpu_usb='xhost + && docker run -it --net=host --gpus all  --env="NVIDIA_DRIVER_CAPABILITIES=all"     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1" --privileged -v /dev/:/dev/ --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/jy/docker/kinova-noetic-gpu/catkin_ws":"/root/catkin_ws":rw    kinova-noetic-gpu bash'

alias kinova_noetic_gpu='xhost + && docker run -it --net=host --gpus all  --env="NVIDIA_DRIVER_CAPABILITIES=all"     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/jy/docker/kinova-noetic-gpu/catkin_ws":"/root/catkin_ws":rw    kinova-noetic-gpu bash'
