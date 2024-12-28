#!/bin/bash

# Vars
custom_fc_appimage=FreeCAD_1.0.0-conda-Linux-x86_64-py311.AppImage
custom_command="./../freecad/freecad_custom_appimage_dir/$custom_fc_appimage --appimage-extract-and-run"
use_default_stable_freecad=true # set false if you want to use some other FreeCAD than default stable FreeCAD (default FC starts faster).
# Dont forget place FC to docker/freecad/freecad_custom_appimage_dir in that case and fix $custom_fc_appimage variable value
command=freecad
force_run_new_container=false

ros_distro=jazzy
ros_distro_assemble=desktop
ws_dir_name=ros2_ws_with_freecad
ros_container_name=ros2_${ros_distro}_with_freecad
image=osrf/ros:$ros_distro-$ros_distro_assemble-with_overcross_deps
parent_dir_of_ws_dir_name=/ros2
freecad_ros2_package_with_deps=freecad_cross_rosdep


# Prepare paths
cont_user_path=/home/$USER
cont_path_ws=$cont_user_path/$parent_dir_of_ws_dir_name/$ws_dir_name
basedir=`dirname $0`
script_dir=`cd $basedir; pwd; cd - > /dev/null 2>&1;`
root_of_freecad_robotcad=$script_dir/../
ws_path=$root_of_freecad_robotcad/docker/ros2_ws/
build_data_path=$ws_path/build_data


# Usage info
show_help() {
cat << EOF
Usage: ${0##*/} [-dfh]
Run RobotCAD in container and open it window at host.

    -h          display this help and exit
    -f          force run new container (remove old one first if present also recreate build dirs)
    -d          debug
EOF
}

# process params of script
while getopts dfh opt; do
    case $opt in
        h)
            show_help
            exit 0
            ;;
        f)
            force_run_new_container=true
            echo 'Force run new container.'
            ;;
        d)
            debug=true
            echo 'DEBUG is active.'
            ;;
        *)
            show_help >&2
            exit 1
            ;;
    esac
done



echo 'Paths: '
echo '$script_dir: '$script_dir
echo '$ws_path: '$ws_path
echo '$cont_path_ws: '$cont_path_ws
echo '$root_of_freecad_robotcad: '$root_of_freecad_robotcad
echo ''

if [ "$use_default_stable_freecad" = false ] ; then
    command=$custom_command
fi
echo "Command to run in container: $command"


[ "$force_run_new_container" = true ] && [ -d $build_data_path ] && rm -rf $build_data_path # remove build dirs if new container
# make dirs at host for builds, logs, other from container
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
        --build-arg UID=$(id -u $USER) \
        --build-arg GROUP=$(groups | awk '{print $1}') \
        --build-arg GID=$(id -g $USER) \
        --build-arg ROS_DISTRO_ARG=$ros_distro \
        --build-arg ROS_DISTRO_ASSEMBLY_ARG=$ros_distro_assemble \
        --build-arg CONT_PATH_WS=$cont_path_ws \
        .
fi


# remove current container if want to run new one
if [ "$force_run_new_container" = true ]; then
    echo "Stop and remove old container if present."
    # stop runnin container(s)
    docker ps -q --filter "name=$ros_container_name" | xargs -r docker stop > /dev/null
    # remove existing container(s)
    docker ps -aq --filter "name=$ros_container_name" | xargs -r docker rm > /dev/null
fi


# Restart if container already run otherwise start it
if [[ $(docker ps -q --filter name=$ros_container_name) ]]; then

    echo "Restart container."
    docker restart $ros_container_name

elif [[ $(docker ps -aq --filter name=$ros_container_name) ]]; then

    echo "Start container."
    docker start $ros_container_name

else

    # If Nvidia is Prime (main video card) it will add Nvidia container options (must be installed nvidia-container-toolkit)
    # [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
    # If dont need this set Prime integrated video (f.e. sudo prime-select intel). It usually is default.
    nvidia_options=''
    if [ "$(prime-select query 2> /dev/null)" == 'nvidia' ]; then
        nvidia_options='--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all'
    fi


    debug_port=''
    debug_env=''
    localhost_address=''
    if [ "$debug" = true ] ; then
        debug_port='-p 5678:5678'
        debug_env='DEBUG=1'
        localhost_address='--add-host localhost=172.17.0.1' # 172.17.0.1 - docker host OC address
    fi

    xhost +local:
    mount_options=',type=volume,volume-driver=local,volume-opt=type=none,volume-opt=o=bind'
    docker run -t -d --name=$ros_container_name \
        --workdir=$cont_path_ws \
        --mount dst=$cont_path_ws/build,volume-opt=device=$build_data_path/build$mount_options \
        --mount dst=$cont_path_ws/install,volume-opt=device=$build_data_path/install$mount_options \
        --mount dst=$cont_path_ws/log,volume-opt=device=$build_data_path/log$mount_options \
        --mount dst=$cont_user_path/.ros/log,volume-opt=device=$build_data_path/ros2_system_logs$mount_options \
        --volume=$root_of_freecad_robotcad/docker/ros2_ws/src:$cont_path_ws/src \
        --volume=$HOME/.local/share/FreeCAD/Mod:$cont_user_path/.local/share/FreeCAD/Mod \
        --volume=$root_of_freecad_robotcad:$cont_user_path/.local/share/FreeCAD/Mod/freecad.robotcad \
        --volume=$root_of_freecad_robotcad/docker/freecad:$cont_path_ws/../freecad \
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
        $debug_port \
        $localhost_address \
        $image bash -c ". ~/.profile && $debug_env $command"
    xhost -


    echo "Ran new container."

fi


echo -e "\nFreeCAD mods from host binded to container and will be accessible.\n"

GREEN='\033[0;32m'
NC='\033[0m' # No Color
echo -e "\n\n\n${GREEN}FreeCAD was started inside container. Wait it window load to your host... (about 20 seconds)\n
The first time, dependencies will be downloaded and installed via pip, which will take longer.${NC}\n\n\n"
