# (optional) If you dont have docker installed

## Install Docker with Compose and Buildx plugins
https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

### Add your user to docker group
```
sudo usermod -aG docker $USER
```

### Reboot system
```
reboot
```

# Run FreeCAD with RobotCAD
```
bash run.bash
```

It will build image and run FreeCAD in container then open FreeCAD window at host. Also it bind host FreeCAD mods to container.

Folder bindings (cont <-> host):

<repo>/docker/freecad/freecad_projects_save_place/ - folder for your FC projects 

<repo>/docker/freecad/freecad_appimage_dir - folder where you can place FreeCAD appImage if you want not to use default stable FreeCAD

<repo>/docker/ros2_ws/src - ROS2 workspace src inside container

<repo>/docker/ros2_ws/build_data - ROS2 workspace build folders (build, install, log, etc) inside container

~/.local/share/FreeCAD/Mod - mods of FreeCAD


# Troubleshooting

## ERROS when run FreeCAD:
"Program received signal SIGSEGV, Segmentation fault."
Mean not enough rights with some folder that FreeCAD using. FreeCAD can be run with sudo or find folder (usualy ~/.local/share/FreeCAD) and "chown" it to current user
