from pathlib import Path

from setuptools import setup

package_name = 'lidar_feature_launch'


def get_launch_paths():
    dirpath = Path('share', package_name, 'launch')
    names = Path('launch').glob('*.launch.py')
    return (str(dirpath), list(map(str, names)))


def get_config_paths():
    dirpath = Path('share', package_name, 'config')
    names = Path('config').glob('*.param.yaml')
    return (str(dirpath), list(map(str, names)))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        get_launch_paths(),
        get_config_paths(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takeshi Ishita',
    maintainer_email='ishitah.takeshi@gmail.com',
    description='Launcher',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
