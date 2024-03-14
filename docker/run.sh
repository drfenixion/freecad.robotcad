#!/bin/sh

# Vars
ros_distro=humble
ws_dir_name=ros2_ws_with_freecad
ros_container_name=ros2_${ros_distro}_with_freecad
image=osrf/ros:$ros_distro-desktop
parent_dir_of_ws_dir_name=/ros2 #/ros2/ros2_ws_with_freecad
freecad_ros2_package_with_deps=freecad_cross_rosdep

# Prepare paths
cont_user_path=/root
cont_path_ws=$parent_dir_of_ws_dir_name/$ws_dir_name
script_dir="$(cd "$(dirname "$1")"; pwd -P)"
root_of_freecad_cross=$script_dir/../
ws_path=$script_dir/../../$ws_dir_name

[ -d $ws_path ] || mkdir -p $ws_path

echo 'Paths: '
echo '$script_dir: '$script_dir
echo '$ws_path: '$ws_path
echo '$cont_path_ws: '$cont_path_ws
echo ''

for dir in 'build' 'install' 'log' 'ros2_system_logs'
do
    [ -d $ws_path/$dir ] || mkdir -p $ws_path/$dir
done

xhost +local:
mount_options=',type=volume,volume-driver=local,volume-opt=type=none,volume-opt=o=bind'
docker run -t -d --name=$ros_container_name \
    --workdir=$cont_path_ws \
    --mount dst=$cont_path_ws/build,volume-opt=device=$ws_path/build$mount_options \
    --mount dst=$cont_path_ws/install,volume-opt=device=$ws_path/install$mount_options \
    --mount dst=$cont_path_ws/log,volume-opt=device=$ws_path/log$mount_options \
    --mount dst=$cont_user_path/.ros/log,volume-opt=device=$ws_path/ros2_system_logs$mount_options \
    --volume=$ws_path/src:$parent_dir_of_ws_dir_name/$ws_dir_name/src \
    --volume=$script_dir/$freecad_ros2_package_with_deps:$parent_dir_of_ws_dir_name/$ws_dir_name/src/$freecad_ros2_package_with_deps \
    --volume=$root_of_freecad_cross:$cont_user_path/.local/share/FreeCAD/Mod/freecad.cross \
    --volume=$root_of_freecad_cross/docker/freecad_appimage_dir:$parent_dir_of_ws_dir_name/FreeCAD \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev/dri:/dev/dri" \
    --privileged \
    --env "DISPLAY=$DISPLAY" \
    --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env PULSE_SERVER=$PULSE_SERVER \
    --env QT_X11_NO_MITSHM=1 \
    --network=bridge \
    --shm-size=512m \
    --security-opt seccomp=unconfined \
    $image \
    bash
xhost -
