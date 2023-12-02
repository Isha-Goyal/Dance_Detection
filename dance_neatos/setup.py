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
            'teleop = dance_neatos.teleop:main',
            'yolo_video_neato = dance_neatos.yolo_video_neato:main',
        ],
    },
)
