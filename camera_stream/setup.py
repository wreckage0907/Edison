from setuptools import setup

package_name = 'camera_stream'

setup(
    name=package_name,
    version='0.0.1',  # Use a simple version format
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Traffic light detection package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_detector = camera_stream.traffic_light_detector:main',
            'traffic_light_viewer = camera_stream.traffic_light_viewer:main',
            'camera_publisher = camera_stream.camera_publisher:main',
            'camera_viewer  = camera_stream.camera_viewer:main',
        ],
    },
)
