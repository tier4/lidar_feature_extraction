from setuptools import setup

package_name = 'point_type_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takeshi Ishita',
    maintainer_email='ishitah.takeshi@gmail.com',
    description='Point cloud conversion for the extraction module',
    license='BSD3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = point_type_converter.convert:main',
        ],
    }
)
