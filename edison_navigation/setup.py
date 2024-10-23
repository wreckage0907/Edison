from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'edison_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('edison_navigation/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wreckage',
    maintainer_email='girish.raghav2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'gps_handler = edison_navigation.modules.gps_handler:main',
        'traffic_sign_detector = edison_navigation.modules.traffic_sign_detector:main',
        'gps_simulator = edison_navigation.modules.gps_simulator:main',
    ],
},
)
