from setuptools import setup

package_name = 'control_motor_pkg'

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
    zip_safe=True,
    maintainer='aiot',
    maintainer_email='lequanganh6723@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = control_motor_pkg.controlMotor_pub:main',
            'listener = control_motor_pkg.controlMotor_sub:main',
        ],
    },
)
