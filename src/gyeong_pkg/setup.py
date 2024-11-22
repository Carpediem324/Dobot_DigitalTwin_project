from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gyeong_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='gusgkr0324@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobot_mv=gyeong_pkg.dobot_mv:main',
            'yolo=gyeong_pkg.only_yolo:main',
            'sock_yolo=gyeong_pkg.yolo_with_socket:main',
            'robo_dk=gyeong_pkg.robo_dk:main',
        ],
    },
)
