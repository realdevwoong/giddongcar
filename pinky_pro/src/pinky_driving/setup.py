from setuptools import find_packages, setup


package_name = 'pinky_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giddongcar',
    maintainer_email='team@todo.todo',
    description='Pinky Pro 단일 로봇용 차선 추종 실험',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tape_lane_drive = pinky_driving.tape_lane_drive:main',
        ],
    },
)
