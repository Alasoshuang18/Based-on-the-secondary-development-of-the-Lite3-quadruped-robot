from glob import glob
from setuptools import find_packages, setup

package_name = 'lite3_lab_5_2'
setup(
    name=package_name, version='1.0.0', packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
    maintainer='Lite3 Course Team', maintainer_email='course@example.invalid',
    description='Auditable ROS 2 Humble navigation teaching experiment',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': ['experiment = ' + package_name + '.manual_navigation:main', 'auto_experiment = ' + package_name + '.exploration:main', 'session = lite3_lab_5_2.session:main', 'session_registry = lite3_lab_5_2.session_registry:main']},
)
