# RobotCAD also known as FreeCAD OVERCROSS

RobotCAD is a FreeCAD workbench to generate robot description packages (xacro or URDF) for the Robot Operating System, [ROS2].

<a href="https://www.youtube.com/watch?v=PGEWqatcUJM" target="_blank"><img src="https://github.com/user-attachments/assets/f3a1646f-5205-44c9-9167-4e8750541be2" alt="RobotCAD video teaser"/></a>

# Key features short list:
1. Autoinstall and run by startup script
1. Modeling parts (in FreeCAD),
1. Creating robot structure (joints, links, elements of link (Collisions, Visuals, Reals), etc),
1. Сonvenient new tools to set placement of joints and links (intuitive way)
1. Material setting (from library or custom) to link or whole robot
1. Auto calculation (based on material):
    1. mass and inertia
    1. center of mass (in global and local coordinates)
    1. positions of joints relative to the robot's center of mass
1. Collisions automatic making tools (based on Real element of robot link)
1. Basic code generator:
    1. ROS2 package with launchers for Gazebo, RViz
    1. URDF
    1. Meshes
1. Tool for use external extended code generating service
    1. External code generator (you dont need to install ROS2 or Gazebo, this is itself-packaged environment one command to run):
        1. all of basic generator
        1. Project structure
        1. Startup script for build and run docker container with all dependencies of project
        1. Init Git with submodules for dependencies management
        1. Docker related code (dockerfiles, etc)
        1. Specific robot types code (multicopter - PX4 + Gazebo + ROS2)
        1. Nvidia video cards container support
        1. README instruction how to use
1. all features from CROSS workbench

# Fast install and run
If you have docker (buildx, compose plugins) installed just do
```
git clone https://github.com/drfenixion/freecad.overcross.git
cd freecad.overcross/docker
bash run.bash
```
Tested on Ubuntu 22.04.

If docker is not installed look at docker/README.md. There is also additional information on how to use the startup script.

You also can install RobotCAD manually (long way) by [Installation](#Installation) section

# Overview
### Launched RViz and Gazebo using generated code:

RViz
![RViz](https://github.com/user-attachments/assets/4e3f28ca-e798-44a5-9f5f-422c4de770e5)

Gazebo - Basic view
![basic](https://github.com/user-attachments/assets/52aa2789-8cd1-442c-ad64-ae48f297cca7)

Gazebo - Joints view
![joints](https://github.com/user-attachments/assets/8808ea8d-4784-4344-aa85-eaf12ab5917a)

Gazebo - Collisions view
![collision](https://github.com/user-attachments/assets/b7cae031-40cc-41c3-8233-4d56ca29d2c1)

Gazebo - Inertia view
![inertia](https://github.com/user-attachments/assets/71983ee3-3995-47d2-a8dc-516c6c2b8a48)


# One more robot
Tools of RobotCAD (FreeCAD OVERCROSS)
![Tools of RobotCAD](https://github.com/user-attachments/assets/cde5a1b4-7c75-406c-9a8e-c0815eccfce4)

Choosing of material of robot or link
![Choosing of material of robot or link](https://github.com/user-attachments/assets/54593e92-dba6-4280-9510-b77cb2048910)

Generated ROS 2 package

![Generated ROS 2 package](https://github.com/drfenixion/freecad.overcross/assets/13005708/f366c2d2-af67-46e2-b7ea-8d03821e5646)

Launched Rviz and Gazebo from generated Gazebo launcher
![Launched Rviz and Gazebo from gazebo launcher](https://github.com/drfenixion/freecad.cross/assets/13005708/9017aec4-70e5-45fa-82ad-6b4646453767)

Generated inertia blocks and centers of mass in Gazebo
![Generated inertia blocks and centers of mass in Gazebo](https://github.com/drfenixion/freecad.cross/assets/13005708/a46715a0-0dc6-4f6e-b80e-e4c644589477)

Generated collisions in Gazebo
![Generated collisions in Gazebo](https://github.com/drfenixion/freecad.cross/assets/13005708/c43a8d29-fe17-4268-b0dc-76f943c4b0b5)

## Usage

[Common usage plan](https://github.com/drfenixion/freecad.overcross/wiki) 

## Description

RobotCAD is a powerful ROS workbench for [FreeCAD](https://www.freecad.org/), a popular open-source 3D parametric modelling software.
As the field of robotics continues to evolve rapidly, the need for comprehensive and efficient tools for robot development and simulation has become increasingly essential.
RobotCAD emerges as a versatile solution, empowering engineers, researchers, and hobbyists to leverage the capabilities of both ROS and FreeCAD in a cohesive environment.
At the time of writing (June 2023), RobotCAD is the only available open-source solution to generate robot description files for ROS with a graphical user interface with direct visual feedback.

With RobotCAD, users gain the ability to combine the flexibility of FreeCAD's 3D modeling capabilities with the extensive functionality of ROS, allowing for seamless collaboration between mechanical design and robotics development.
    By bridging the gap between these two powerful platforms, RobotCAD streamlines the process of designing, and visualizing robotic systems, ultimately accelerating the development cycle.

The key features of RobotCAD are:

* ROS Integration: RobotCAD offers native integration with ROS, an open-source framework widely adopted in the robotics community.
        This integration enables users to leverage the vast ecosystem of ROS packages, libraries, and tools while working within the familiar FreeCAD environment.
* 3D Modelling and Simulation: FreeCAD's 3D modelling capabilities empower users to design intricate mechanical components and complete robot systems.
        With RobotCAD, these designs can be seamlessly integrated with ROS simulations, allowing for realistic and accurate testing of robot behaviors and interactions.
* Visualization and Analysis: RobotCAD provides advanced visualization and analysis tools provided by FreeCAD itself, enabling users to inspect, analyze, and validate their robot designs.
* Collaborative Development: RobotCAD supports collaborative development by facilitating the sharing of robot models through the use of complex macro written in the Python language where a full robot can be generated by code.
        This encourages teamwork, knowledge sharing, and accelerates the pace of innovation within the robotics community.
* Extensibility: As an open-source project, RobotCAD encourages contributions from the community, allowing users to extend its functionality and adapt it to their specific needs.
        By leveraging the collective expertise of the ROS and FreeCAD communities, RobotCAD continues to evolve and provide cutting-edge features for robot development.

## Compatibility

Compatible with FreeCAD at least v0.21.2.
Compatible with ROS2.

## Features

- Export `Part::Box`, `Part::Sphere`, and `Part::Cylinder` as text to be included in a URDF file,
- Generate an enclosing box or sphere as collision object (only axis-aligned box for now),
- Build a robot from scratch and generate the URDF file for it,
- Set a value for each actuated joint of a robot and have the links move accordingly,
- Import URDF/xacro files,
- Import xacro definitions, i.e. import xacro files that only define some macros and ask the user to choose a macro and its parameters to generate a full-feature URDF,
- Export the xacro as xacro that includes (`xacro:include`) the original xacro file and uses the macro.
- Combine several xacros files (i.e. also URDF) into a workcell and export them as a xacro file.
- Get the current planning scene (relies on the /get_planning_scene service of type `moveit_msgs/srv/GetPlanningScene`)
- Define a pose and possibly bring a specific link to it. All links that are fixed to this link will follow but the inverse kinematic solutions are not shown.

## Installation 

You need a recent version of FreeCAD v0.21.2 with the ability to configure custom repositories for the Addon Manager to install the workbench via the Addon Manager. On earlier version you're on your own, see instructions for local install below.

- In FreeCAD, menu "Edit / Preferences ..."
- Category "Addon Manager"
- Add an entry to "Custom repository" by clicking on the "+" sign.
- Repository URL: `https://github.com/drfenixion/freecad.overcross.git`, branch: `main`
- Click on "OK" to close the dialog "Preferences"
- Back to FreeCAD's main window, menu "Tools / Addon manager"
- Search and install the workbench via the [Addon Manager](https://wiki.freecad.org/Std_AddonMgr)

## Launching FreeCAD with ROS

The RobotCAD workbench is supposed to load also without ROS, with limited functionality.
If this is not the case, please report.

You will probably want to be able to use ROS-related functionalities and this requires launching FreeCAD from the command line:

- Install all RobotCAD dependencies by install RobotCAD dependency meta package
- `sudo apt-get update`
- `cd docker/ros2_ws/ && rosdep update && rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}`
- `. /install/setup.bash`

(Optional) You can also set extra Python modules
- Open a terminal
- Source your ROS workspace
- Launch FreeCAD with extra Python modules set by ROS, `freecad --module-path ${PYTHONPATH//:/' --module-path '}` (replace `freecad` by your FreeCAD executable). This bash magic will add for example ` --module-path path1 --module-path path2 ` if `$PYTHONPATH` is `path1:path2`.

## Testing/developing the workbench

If you want to work on this workbench you have the following options:

- Clone the repository directory in FreeCAD's `Mod` directory: `cd ~/.local/share/FreeCAD/Mod && git clone https://github.com/drfenixion/freecad.overcross.git` on Linux
- Start FreeCAD from the root-directory of this repository in a terminal (by default `freecad.overcross`)
- Clone this repository and create a symbolic link to the directory `freecad.overcross` (or the directory containing this repository if you changed the name) to FreeCAD's `Mod` directory (`~/.local/share/FreeCAD/Mod` on Linux).
- `pip install -e .` adds the root-directory to `easy_install.path`.


--------------------------------------------------------------------------------

[ROS2]: https://www.ros.org/
