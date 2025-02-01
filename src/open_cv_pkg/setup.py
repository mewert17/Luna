from setuptools import find_packages, setup

package_name = 'open_cv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[  # Optional: for package resource files
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mewert',
    maintainer_email='mewert@todo.todo',
    description='Package for object detection and occupancy grid generation from point cloud',
    license='MIT',  # Specify the correct license here
    tests_require=['pytest'],
    entry_points={  # Add the entry for the new node
        'console_scripts': [
            'object_detection_node = open_cv_pkg.object_detection:main',  # Existing node
            'og_from_pointcloud = open_cv_pkg.og_from_pointcloud:main',  # New node
        ],
    },
)
