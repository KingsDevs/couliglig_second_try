from setuptools import find_packages, setup
from glob import glob

package_name = 'couliglig'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', glob('launch/*.py')))
data_files.append(('share/' + package_name + '/worlds', glob('worlds/*.wbt')))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Robec_24_25_Banner.png']))
data_files.append(('share/' + package_name + '/worlds/stl_files', glob('worlds/stl_files/*.STL')))
data_files.append(('share/' + package_name + '/resource', glob('resource/*.urdf')))
data_files.append(('share/' + package_name + '/config', glob('config/*.yaml')))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minegears',
    maintainer_email='minegears@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'couliglig_bot = couliglig.couliglig_bot:main',
            'keyboard_controller = couliglig.keyboard_controller:main',
        ],
    },
)
