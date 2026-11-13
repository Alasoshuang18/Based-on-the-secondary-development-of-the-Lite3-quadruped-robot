from setuptools import setup
from glob import glob
package_name = 'lite3_lab_3_1'
setup(name=package_name, version='1.0.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml', 'README.md']),
                  ('share/' + package_name + '/launch', glob('launch/*.py')),
                  ('share/' + package_name + '/config', glob('config/*')),
                  ('share/' + package_name + '/rviz', glob('rviz/*'))],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      maintainer='Course Maintainer', maintainer_email='course@example.com',
      description='Runnable sensor laboratory for ROS 2 Humble', license='Apache-2.0',
      entry_points={'console_scripts': ['experiment = '+package_name+'.experiment:main']})
