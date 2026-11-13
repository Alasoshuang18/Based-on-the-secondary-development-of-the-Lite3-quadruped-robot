from glob import glob
from setuptools import setup

package_name = 'lite3_lab_4_1'
setup(name=package_name, version='1.0.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml', 'README.md']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml')),
                  ('share/' + package_name + '/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools', 'numpy', 'PyYAML', 'Pillow'], zip_safe=True,
      maintainer='三点智联课程组', maintainer_email='course@example.com',
      description='机器狗第四章实验 lite3_lab_4_1', license='Apache-2.0',
      entry_points={'console_scripts': ['experiment = lite3_lab_4_1.experiment:main']})

