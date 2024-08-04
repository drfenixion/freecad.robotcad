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
    maintainer='galou',
    maintainer_email='gael.ecorchard@cvut.cz',
    url='https://github.com/galou/freecad.cross.git',
    description='CROSS, a workbench to work with ROS in FreeCAD',
    install_requires=[],
    include_package_data=True,
)

# install_requires should be ['xacro'].
