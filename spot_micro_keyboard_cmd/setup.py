from setuptools import setup
from glob import glob

package_name = 'spot_micro_keyboard_cmd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike@example.com',
    description='The spot_micro_keyboard_cmd package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'spot_micro_keyboard_move = spot_micro_keyboard_cmd.scripts.spotMicroKeyboardMove:main',
        ],
    },
)
