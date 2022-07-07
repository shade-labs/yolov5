from setuptools import setup

package_name = 'yolov5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'torch'],

    zip_safe=True,
    maintainer='Shade',
    maintainer_email='jeremy@shaderobotics.com',
    description='ROS2 wrapper for yolov5',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface = yolov5.interface:main'
        ],
    },
)
