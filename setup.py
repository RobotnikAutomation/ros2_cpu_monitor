from setuptools import setup

package_name = 'temperature_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # maintainer='colin',
    # maintainer_email='colin.fuelberth@icloud.com',
    maintainer='Guillem Gari',
    maintainer_email='guillem.gari@robotnik.es',
    parameters=[
        {'publish_cpu_temperature': True},
        {'cpu_type_id': 'x86_pkg_temp'},
        {'publish_rate': 1}
    ],
    description='CPU monitor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpu_monitor = cpu_monitor.cpu_monitor:main'
        ],
    },
)
