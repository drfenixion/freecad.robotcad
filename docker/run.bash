#!/bin/bash

# Vars
custom_fc_appimage=FreeCAD-0.21.2-Linux-x86_64.AppImage
custom_command="./../freecad/freecad_custom_appimage_dir/$custom_fc_appimage --appimage-extract-and-run"
use_default_stable_freecad=true # set false if you want to use some other FreeCAD than default stable FreeCAD (default FC starts faster).
# Dont forget place FC to docker/freecad/freecad_custom_appimage_dir in that case and fix $custom_fc_appimage variable value
command=freecad

ros_distro=iron
ros_distro_assemble=desktop
ws_dir_name=ros2_ws_with_freecad
ros_container_name=ros2_${ros_distro}_with_freecad
image=osrf/ros:$ros_distro-$ros_distro_assemble-with_overcross_deps
parent_dir_of_ws_dir_name=/ros2
freecad_ros2_package_with_deps=freecad_cross_rosdep


# Prepare paths
cont_user_path=/home/$USER
cont_path_ws=$cont_user_path/$parent_dir_of_ws_dir_name/$ws_dir_name
script_dir="$(cd "$(dirname "$1")"; pwd -P)"
root_of_freecad_cross=$script_dir/../
ws_path=$root_of_freecad_cross/docker/ros2_ws/
build_data_path=$ws_path/build_data

echo 'Paths: '
echo '$script_dir: '$script_dir
echo '$ws_path: '$ws_path
echo '$cont_path_ws: '$cont_path_ws
echo '$root_of_freecad_cross: '$root_of_freecad_cross
echo ''

if [ "$use_default_stable_freecad" = "false" ]
then
    command=$custom_command
fi
echo "command: $command"

# make dirs at host for builds, logs, other from container
[ -d $build_data_path ] && rm -rf $build_data_path # remove dir
for dir in 'build' 'install' 'log' 'ros2_system_logs'
do
    [ -d $build_data_path/$dir ] || mkdir -p $build_data_path/$dir
done


# build if image not exists
if [ -z "$(docker images -q $image 2> /dev/null)" ]; then
    echo 'Build docker container...'
    # build ROS image
    docker buildx build -t $image --shm-size=512m \
        --build-arg USER=$USER \
        --build-arg ROS_DISTRO_ARG=$ros_distro \
        --build-arg ROS_DISTRO_ASSEMBLY_ARG=$ros_distro_assemble \
        --build-arg CONT_PATH_WS=$cont_path_ws \
        .
fi


# stop runnin container(s)
docker ps -q --filter "name=$ros_container_name" | xargs -r docker stop > /dev/null
# remove existing container(s)
docker ps -aq --filter "name=$ros_container_name" | xargs -r docker rm > /dev/null


# If Nvidia is Prime (main video card) it will add Nvidia container options (must be installed nvidia-container-toolkit)
# [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
# If dont need this set Prime integrated video (f.e. sudo prime-select intel). It usually is default.
nvidia_options=''
if [ "$(prime-select query 2> /dev/null)" == 'nvidia' ]; then 
    nvidia_options='--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all'
fi


xhost +local:
mount_options=',type=volume,volume-driver=local,volume-opt=type=none,volume-opt=o=bind'
docker run -t -d --name=$ros_container_name \
    --workdir=$cont_path_ws \
    --mount dst=$cont_path_ws/build,volume-opt=device=$build_data_path/build$mount_options \
    --mount dst=$cont_path_ws/install,volume-opt=device=$build_data_path/install$mount_options \
    --mount dst=$cont_path_ws/log,volume-opt=device=$build_data_path/log$mount_options \
    --mount dst=$cont_user_path/.ros/log,volume-opt=device=$build_data_path/ros2_system_logs$mount_options \
    --volume=$root_of_freecad_cross/docker/ros2_ws/src:$cont_path_ws/src \
    --volume=$HOME/.local/share/FreeCAD/Mod:$cont_user_path/.local/share/FreeCAD/Mod \
    --volume=$root_of_freecad_cross:$cont_user_path/.local/share/FreeCAD/Mod/freecad.cross \
    --volume=$root_of_freecad_cross/docker/freecad:$cont_path_ws/../freecad \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    $nvidia_options \
    --env DISPLAY=$DISPLAY \
    --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env PULSE_SERVER=$PULSE_SERVER \
    --env QT_X11_NO_MITSHM=1 \
    --network=bridge \
    --shm-size=512m \
    --security-opt seccomp=unconfined \
    $image $command
xhost -

#  -c ". ./install/setup.bash && ./../freecad/freecad_custom_appimage_dir/$custom_fc_appimage --appimage-extract-and-run"
# bash", "-c", ". ./install/setup.bash && ./../freecad/freecad_custom_appimage_dir/FreeCAD-0.21.2-Linux-x86_64.AppImage --appimage-extract-and-run


echo -e "\nFreeCAD mods from host binded to container and will be accessible.\n"

GREEN='\033[0;32m'
NC='\033[0m' # No Color
echo -e "\n\n\n${GREEN}FreeCAD was started inside container. Wait it window load to your host... (about 20 seconds)${NC}\n\n\n"