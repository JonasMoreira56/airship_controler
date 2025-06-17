from setuptools import find_packages, setup

package_name = 'gym-airship'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'gymnasium',  # Use gymnasium para versões mais recentes do Gym
        'numpy',
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'std_srvs'        
        ],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='jonasmoreira076@gmail.com',
    description='Um ambiente Gym para controle de pouso de dirigível no Gazebo.n',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "airship_landing_env=gym_airship.envs.airship_landing_env:AirshipLandingEnv",
        ],
    },
)
