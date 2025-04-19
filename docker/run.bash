#!/bin/bash

# Vars
# For using of ROS packages from inside RobotCAD, FreeCAD build must have same Python version as ROS2 OS (py3.12)
# Some of RobotCAD tools uses ROS2 packages
custom_fc_appimage=FreeCAD_1.0.0-conda-Linux-x86_64-py311.AppImage
custom_command="./../freecad/freecad_custom_appimage_dir/$custom_fc_appimage --appimage-extract-and-run"
use_custom_command=false # set true if you want to use FreeCAD AppImage instead of system (inside docker container) FreeCAD
# Dont forget place FC to docker/freecad/freecad_custom_appimage_dir in that case and fix $custom_fc_appimage variable value
command='. /home/$USER/miniconda3/bin/activate && conda activate freecad_1_0_312 && QT_QPA_PLATFORM=xcb freecad' # py3.12 # QT_QPA_PLATFORM=xcb -for use x11
# command=freecad-daily # latest dev freecad

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
Usage: ${0##*/} [-dfbclonh]
Run RobotCAD in container and open it window at host.

    -h          display this help and exit
    -f          force run new container (remove old one first if present also recreate build dirs)
    -b          force build new image and container
    -c          clear old container logs (required sudo)
    -l          run last dev FreeCAD version (freecad-daily) instead of stable. Dont use if you dont know what is it.
                If you will came back after to stable version add -f option.
    -o          Change owner of FreeCAD share directory to current user (can fix "Segmentation fault" in some cases)
    -n          Force add Nvidia container toolkit options (--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all) (can fix "failed to create drawable" in some cases)
    -d          debug
EOF
}

# process params of script
while getopts dfbclonh opt; do
    case $opt in
        h)
            show_help
            exit 0
            ;;
        f)
            force_run_new_container=true
            echo 'Force run new container is requested.'
            ;;
        d)
            debug=true
            echo 'DEBUG is requested.'
            ;;
        b)
            force_build_new_image=true
            force_run_new_container=true
            echo 'Force build new image and run new container are requested.'
            ;;
        c)
            clear_old_logs=true
            echo 'Clearing old logs is requested.'
            ;;
        l)
            force_run_new_container=true
            command=freecad-daily
            echo 'Run freecad-daily (last dev) is requested.'
            ;;
        o)
            fix_freecad_dirs_owner=true
            echo 'Changing of FreeCAD ~/.local/share directory owner is requested.'
            ;;
        n)
            force_add_nvidia_options=true
            echo 'Force add Nvidia container options is requested.'
            ;;
        *)
            show_help >&2
            exit 1
            ;;
    esac
done


echo ''
echo 'Paths: '
echo '$script_dir: '$script_dir
echo '$ws_path: '$ws_path
echo '$cont_path_ws: '$cont_path_ws
echo '$root_of_freecad_robotcad: '$root_of_freecad_robotcad
echo ''


# Docker install
# Check if Docker is not installed
if [ -z "$(command -v docker)" ]; then

    echo ''
    echo 'Docker is not installed.'
    echo ''

    # check script in Ubuntu OS
    if [ "$(lsb_release -si 2> /dev/null)" == "Ubuntu" ]; then
        echo 'Starting Docker installation... (required password for sudo).'

        # Add Docker's official GPG key:
        sudo apt-get update
        sudo apt-get install ca-certificates curl -y
        sudo install -m 0755 -d /etc/apt/keyrings
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc

        # Add the repository to Apt sources:
        echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        sudo apt-get update

        # Install the Docker:
        sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

        # Add current user to Docker group
        echo 'Adding current user to Docker group...'
        sudo usermod -aG docker $USER

        # Restart Docker service
        echo 'Restarting Docker service...'
        sudo systemctl restart docker

        echo ''
        echo 'Docker installed successfully! In case of any problem with it reboot computer.'
        echo ''

        docker_was_installed_and_host_not_rebooted=true

    else
        echo 'Installation does not proceed in Ubuntu. Installation is interupted.'
        echo 'Install Docker manually for your Linux distribution and run this script again. https://docs.docker.com/engine/install/'
        exit 1
    fi
fi
# END Docker install


# Temporary change user group
# used only after docker install for non-rebooted usescase
if [ $(id -gn) != "docker" ] && [ "$docker_was_installed_and_host_not_rebooted" = true ]; then
    # script self-run with docker group of user
    # used for run docker without reboot after intall

    # Construct an array which quotes all the command-line parameters.
    arr=("${@/#/\"}")
    arr=("${arr[*]/%/\"}")
    exec sg docker "OLD_GID=$(id -g) OLD_GROUP=$(id -gn) bash $0 ${arr[@]}"
fi


if [ "$use_custom_command" = true ] ; then
    command=$custom_command
fi
echo "Command to run in container: $command"


[ "$force_run_new_container" = true ] && [ -d $build_data_path ] && rm -rf $build_data_path # remove build dirs if new container
# make dirs at host for builds, logs, other from container
for dir in 'build' 'install' 'log' 'ros2_system_logs'
do
    [ -d $build_data_path/$dir ] || mkdir -p $build_data_path/$dir
done
#'usr_lib_python-major-ver_dist-packages' 'usr_local_lib_python_ver_dist-packages'


# remove current container if want to run new one
if [ "$force_run_new_container" = true ]; then
    echo "Stop and remove old container if present."
    # stop runnin container(s)
    docker ps -q --filter "name=$ros_container_name" | xargs -r docker stop > /dev/null
    # remove existing container(s)
    docker ps -aq --filter "name=$ros_container_name" | xargs -r docker rm > /dev/null
fi


# remove image if you want to rebuild it
if [ "$force_build_new_image" = true ]; then
    echo 'Remove image.'
    docker image rm -f $image
fi


# build if image not exists
if [ -z "$(docker images -q $image 2> /dev/null)" ]; then
    echo 'Build docker container...'

    echo ''
    echo 'Docker image will take about 12Gb free space. Check you have more.'
    echo ''

    uid=$(id -u $USER)
    # Use old user group in case of group was changed after docker install
    # or use current user group if installion without docker install
    gid=${OLD_GID:-$(id -g $USER)}
    group=${OLD_GROUP:-$(groups | awk '{print $1}')}

    echo "UID will be used for user inside image:  $uid"
    echo "User name will be used for user inside image:  $USER"
    echo "GID will be used for user inside image:  $gid"
    echo "GROUP will be used for user inside image:  $group"

    # build ROS image
    docker buildx build -t $image --shm-size=512m \
        --build-arg USER=$USER \
        --build-arg UID=$uid \
        --build-arg GROUP=$group \
        --build-arg GID=$gid \
        --build-arg ROS_DISTRO_ARG=$ros_distro \
        --build-arg ROS_DISTRO_ASSEMBLY_ARG=$ros_distro_assemble \
        --build-arg CONT_PATH_WS=$cont_path_ws \
        .
fi


# Remove old container logs
if [ "$clear_old_logs" = true ] ; then
    log_path=$(docker inspect --format='{{.LogPath}}' $ros_container_name 2> /dev/null)
    if [ -n "$log_path" ]; then
        echo "Clear old container logs. (required password for sudo)"
        sudo truncate -s 0 "$log_path"
    else
        echo "Logs of '$ros_container_name' container are not exists. Nothing to clear."
    fi
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
    if [ "$(prime-select query 2> /dev/null)" == 'nvidia' ] || [ "$force_add_nvidia_options" = true ]; then
        nvidia_options='--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all'
    fi


    debug_port=''
    debug_env=''
    localhost_address=''
    if [ "$debug" = true ] ; then
        debug_port='-p 5678:5678'
        debug_env='export DEBUG=1 &&'
        localhost_address='--add-host localhost=172.17.0.1' # 172.17.0.1 - docker host OC address
    fi


    host_freecad_mods_path=$HOME/.local/share/FreeCAD/Mod
    host_freecad_share_path=$HOME/.local/share/FreeCAD
    cont_freecad_mods_path=$cont_user_path/.local/share/FreeCAD/Mod
    cont_freecad_share_path=$cont_user_path/.local/share/FreeCAD
    host_freecad_user_config_path=$HOME/.FreeCAD
    cont_freecad_user_config_path=$cont_user_path/.FreeCAD
    host_freecad_cache_path=$HOME/.cache/FreeCAD
    cont_freecad_cache_path=$cont_user_path/.cache/FreeCAD

    if [ ! -d "$host_freecad_mods_path" ]; then
        # Create the directory and its parents if they don't exist
        echo "Create host freecad mods directory."
        mkdir -p "$host_freecad_mods_path"
    fi


    if [ "$fix_freecad_dirs_owner" = true ]; then
        echo 'Change "~/.local/share/FreeCAD" directory owner to current user. (required password for sudo)'
        # Set the ownership of the created directory to the current user
        sudo chown -R $USER:$USER "$host_freecad_share_path"
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
        --volume=$host_freecad_share_path:$cont_freecad_share_path \
        --volume=$host_freecad_user_config_path:$cont_freecad_user_config_path \
        --volume=$host_freecad_cache_path:$cont_freecad_cache_path \
        --volume=$root_of_freecad_robotcad:$host_freecad_mods_path/freecad.robotcad \
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
        $image bash -c ". \${HOME}/.profile && $debug_env $command"
    xhost -
    # --volume=$HOME/.local/share/FreeCAD/Mod:$cont_user_path/.local/share/FreeCAD/Mod \
    # --mount dst=/usr/lib/python3/dist-packages,volume-opt=device=$build_data_path/usr_lib_python-major-ver_dist-packages$mount_options \
    # --mount dst=/usr/local/lib/python3.12/dist-packages,volume-opt=device=$build_data_path/usr_local_lib_python_ver_dist-packages$mount_options \

    echo "Ran new container."

fi


echo -e "\nFreeCAD mods from host binded to container and will be accessible.\n"

GREEN='\033[0;32m'
NC='\033[0m' # No Color
echo -e "\n\n\n${GREEN}FreeCAD was started inside container. Wait it window load to your host... (about 20 seconds)\n
The first time, dependencies will be downloaded and installed via pip, which will take longer.${NC}\n\n\n"


# Echo container logs to host console if container run
if [[ $(docker ps -q --filter name=$ros_container_name) ]]; then

    echo "Container $ros_container_name is run."
else
    echo "Container $ros_container_name is not run. Some problem occurred. See container logs."
fi

echo -e "Echo container logs.\n\n\n"
docker logs -f $ros_container_name
