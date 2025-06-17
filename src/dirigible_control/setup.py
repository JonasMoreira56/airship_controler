import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dirigible_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Instala o conteúdo da pasta models
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        # Se tiver meshes ou outros assets, instale-os também:
        # (os.path.join('share', package_name, 'models/meshes'), glob('models/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dirigible_controller = dirigible_control.dirigible_controller:main",
            "train_dqn = dirigible_control.train_dqn:main",
        ],
    },
)
