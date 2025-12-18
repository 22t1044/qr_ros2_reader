from setuptools import find_packages, setup
import os
import glob

package_name = 'qr_ros2_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools', 'pyzbar', 'opencv-python'],
    zip_safe=True,
    maintainer='22t1044',
    maintainer_email='22t1044@stu.meisei-u.ac.jp',
    description='QR code reader ROS 2 package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_code_reader = qr.qr:main',
        ],
    },
)
