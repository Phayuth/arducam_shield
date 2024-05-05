import os
from glob import glob
from setuptools import setup

package_name = 'arducam_shield'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), [os.path.join(path, name) for path, _, files in os.walk('config/') for name in files]),
        (os.path.join('share', package_name, 'libfiles'), glob('libfiles/*')),
        (os.path.join('share', package_name, 'calib_data'), glob('calib_data/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuth',
    maintainer_email='yuth@todo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'arducam = arducam_shield.arducam_rosstream:main'
        ],
    },
)