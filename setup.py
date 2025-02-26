from setuptools import find_packages, setup
import glob

package_name = 'tcebot_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob.glob('meshes/*')),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*')),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dhanush M',
    maintainer_email='dhanushshettigar90@gmail.com',
    description='The tcebot_description package provides URDF/Xacro files for the TCEBot robot model.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
