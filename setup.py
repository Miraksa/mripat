import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mripat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        (os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.pt')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ambatron',
    maintainer_email='ambatron@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'computer_vision = mripat.computerVision:main',
            'object_detection = mripat.objectDetection:main',
            'record_camera = mripat.recordCam:main',
            'camera = mripat.camera:main'
        ],
    },
)
