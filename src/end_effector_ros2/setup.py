from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'end_effector_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
          glob('launch/*.py')),
        (os.path.join('share', package_name, 'meshes'),
          glob('meshes/*')),
        (os.path.join('share', package_name, 'weights'),
          glob('weights/*.pt')),
        # ── Araç şase modeli — sadece dosyalar, klasör değil ─────────────
        (os.path.join('share', package_name, 'models', 'arac_sase'),
          glob('models/arac_sase/*.sdf') + glob('models/arac_sase/*.config')),
        (os.path.join('share', package_name, 'models', 'arac_sase', 'meshes'),
          glob('models/arac_sase/meshes/*.stl')),
        # ─────────────────────────────────────────────────────────────────
        (os.path.join('lib', package_name),
         glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emin',
    maintainer_email='emin@todo.todo',
    description='End Effector Control with YOLO and CAN Bus',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node           = end_effector_ros2.gui_node:main',
            'vision_node        = end_effector_ros2.vision_node:main',
            'logic_node         = end_effector_ros2.logic_node:main',
            'can_node           = end_effector_ros2.can_node:main',
            'cartesian_to_joint = end_effector_ros2.cartesian_to_joint_node:main',
            'gazebo_bridge      = end_effector_ros2.gazebo_bridge:main',
            'movel_test         = end_effector_ros2.movel_test:main',
            'calibrate_camera   = end_effector_ros2.calibrate_camera:main',
            'car_skeleton       = end_effector_ros2.car_skeleton_publisher:main',
        ],
    },
)
