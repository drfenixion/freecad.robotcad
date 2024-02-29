# Copy your FreeCAD-0.21.2-Linux-x86_64.AppImage to freecad_appimage_dir

``cp FreeCAD-0.21.2-Linux-x86_64.AppImage freecad_appimage_dir``

# Run container

``sh run.sh``

# Run freecad in container like:

```
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt-get update && rosdep update && rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
colcon build
source ./install/setup.bash
./../FreeCAD/FreeCAD-0.21.2-Linux-x86_64.AppImage --appimage-extract-and-run
```

# For your convenience you can install Portainer. It is a docker GUI.

