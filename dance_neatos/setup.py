from setuptools import find_packages, setup

package_name = 'dance_neatos'

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
    maintainer='mcranor',
    maintainer_email='mcranor@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_video = dance_neatos.yolo_video:main',
            'yolo_webcam = dance_neatos.yolo_webcam:main',
            'move_neato_two_directions = dance_neatos.move_neato_two_directions:main',
            'move_neato_one_direction = dance_neatos.move_neato_one_direction:main'
        ],
    },
)
