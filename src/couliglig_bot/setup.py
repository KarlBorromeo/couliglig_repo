from setuptools import find_packages, setup
import glob

package_name = 'couliglig_bot'

data_files=[
    ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ('share/' + package_name + '/worlds', ['worlds/Robec_24_25_Banner.png']),
    ('share/' + package_name + '/worlds', glob.glob('worlds/*.wbt')),
    ('share/' + package_name + '/stl_files', glob.glob('stl_files/*')),
    ('share/' + package_name + '/resource', glob.glob('resource/*.urdf')),
    ('share/' + package_name + '/config', glob.glob('config/*')),
    ('share/' + package_name + '/maps', glob.glob('maps/*')),
    ('share/' + package_name + '/dynamic_files', glob.glob('dynamic_files/*')),
    ('share/' + package_name + '/launch/nav2_bringup', glob.glob('launch/nav2_bringup/*.py')),
    ('share/' + package_name + '/params', glob.glob('params/*')),
    ('share/' + package_name, ['package.xml'])
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karl',
    maintainer_email='karl@todo.todo',
    description='Rosify the Couliglig Bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = couliglig_bot.my_keyboard_controller:main',
        ],
    },
)
