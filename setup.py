from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voice_controller_my_robot'
launch_files = glob(os.path.join('launch', '*.launch.py'))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', ['data/rooms_data.json']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools', 'pynput', 'sounddevice', 'wavio', 'speech_recognition', 'gtts', 'json'],
    zip_safe=True,
    maintainer='axel',
    maintainer_email='axelniato@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_activator = voice_controller_my_robot.keyboard_activator:main",
            "voice_recorder = voice_controller_my_robot.voice_recorder:main",
            "pose_publish_from_room_number = voice_controller_my_robot.pose_publish_from_room_number:main"
        ],
    },
)
