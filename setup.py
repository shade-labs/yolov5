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
    install_requires=['setuptools',
                      'torch>= 1.7.0,!=1.12.0',
                      'matplotlib>=3.2.2',
                      'numpy>=1.18.5',
                      'opencv-python>=4.1.1',
                      'Pillow>=7.1.2',
                      'PyYAML>=5.3.1',
                      'requests>=2.23.0',
                      'scipy>=1.4.1',
                      'torchvision>=0.8.1,!=0.13.0',
                      'tqdm>=4.41.0',
                      'protobuf < 4.21.3',
                      'tensorboard>=2.4.1',
                      'pandas>=1.1.4',
                      'seaborn>=0.11.0',
                      'ipython',
                      'psutil',
                      'thop>=0.1.0'
],

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
