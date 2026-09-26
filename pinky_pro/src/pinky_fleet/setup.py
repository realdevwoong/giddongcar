from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_fleet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*.html')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giddongcar',
    maintainer_email='team@todo.todo',
    description='Pinky Pro 2대 관제: 로봇별 Nav2 + 웹 대시보드',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_dashboard = pinky_fleet.fleet_dashboard:main',
        ],
    },
)
