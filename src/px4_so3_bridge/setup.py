from setuptools import setup

package_name = 'px4_so3_bridge'

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
    zip_safe=False,
    maintainer='young',
    maintainer_email='todo@example.com',
    description='Bridge SO3Command to mavros AttitudeTarget',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'so3_to_mavros_bridge = px4_so3_bridge.so3_to_mavros_bridge:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'px4_so3_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='young',
    maintainer_email='8365230+mzyc-001@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'so3_to_mavros_bridge = px4_so3_bridge.so3_to_mavros_bridge:main',
        ],
    },
)
