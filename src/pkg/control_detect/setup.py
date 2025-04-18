from setuptools import setup

package_name = 'control_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aiot',
    maintainer_email='your@email.com',
    description='Control and detection node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = control_detect.control:main',
        ],
    },
)
