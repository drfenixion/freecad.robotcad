import os

from setuptools import setup

# name: this is the name of the distribution.
# Packages using the same name here cannot be installed together

version_path = os.path.join(
    os.path.abspath(os.path.dirname(__file__)),
    'freecad', 'cross', 'version.py',
)
with open(version_path) as fp:
    exec(fp.read())

setup(
    name='freecad.cross',
    version=str(__version__),
    packages=[
        'freecad',
        'freecad.cross',
    ],
    maintainer='Vasily S',
    maintainer_email='it.project.devel@gmail.com',
    url='https://github.com/drfenixion/freecad.overcross.git',
    description='RobotCAD (FreeCAD OVERCROSS) is a workbench to work with ROS in FreeCAD',
    install_requires=[],
    include_package_data=True,
)

# install_requires should be ['xacro'].
