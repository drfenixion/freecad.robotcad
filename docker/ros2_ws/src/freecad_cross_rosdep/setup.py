from setuptools import find_packages, setup

package_name = 'freecad_cross_rosdep'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Package for store freecad.cross workbench dependencies',
    license='LGPL-2.1',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
