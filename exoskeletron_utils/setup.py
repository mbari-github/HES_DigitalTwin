from setuptools import find_packages, setup

package_name = 'exoskeletron_utils'

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
    maintainer='mbari',
    maintainer_email='mbari@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'logger = exoskeletron_utils.logger:main',
        'gui_input = exoskeletron_utils.gui_input_node:main',
        'external_wrench_sine = exoskeletron_utils.external_wrench_sine:main',
        'external_wrench_step = exoskeletron_utils.external_wrench_step:main',    ],
},
)
