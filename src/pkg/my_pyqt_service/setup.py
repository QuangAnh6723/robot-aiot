from setuptools import setup

package_name = 'my_pyqt_service'

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
    maintainer='anh',
    maintainer_email='lequanganh6723@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_service = my_pyqt_service.simple_service:main',
            'simple_client_ui = my_pyqt_service.simple_client_ui:main',
            'wifi_service = my_pyqt_service.wifi_service:main',
            'wifi_client_ui = my_pyqt_service.wifi_client_ui:main',
        ],
    },
)
