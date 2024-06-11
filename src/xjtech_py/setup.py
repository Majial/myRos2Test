from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xjtech_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('lib', package_name), glob(os.path.join('lib', '*.so'))),  # 添加这一行
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='union',
    maintainer_email='union@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xjtech_node = xjtech_py.xjtech_py:main',
        ],
    },
)
