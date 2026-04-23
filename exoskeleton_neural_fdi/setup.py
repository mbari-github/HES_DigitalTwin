from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'exoskeleton_neural_fdi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Marcello Bari',
    maintainer_email='bari.marcello00@gmail.com',
    description='Neural FDI for hand exoskeleton Digital Twin',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodo ROS 2 principale (COLLECT → TRAIN → INFERENCE)
            'fdi_node = exoskeleton_neural_fdi.fdi_node:main',

            # Trainer standalone (uso offline da terminale)
            'trainer = exoskeleton_neural_fdi.trainer:main',
        ],
    },
)