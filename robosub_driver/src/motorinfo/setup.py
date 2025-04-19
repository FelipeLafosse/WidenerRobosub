from setuptools import find_packages, setup

package_name = 'motorinfo'

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
    maintainer='robosub',
    maintainer_email='robosub@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motorauv = motorinfo.motorauv:main',
            'motorauv2 = motorinfo.motorauv2:main',
            'motorcommand = motorinfo.motorcommand:main',
            'motorkey = motorinfo.motorkey:main',
        ],
    },
)
