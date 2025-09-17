from setuptools import setup

package_name = 'auv_control'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'bno_pub = auv_control.bno_pub:main',
        	'bar_pub = auv_control.bar_pub:main',
        	'pid_node = auv_control.pid_node:main', 
        	'thruster_node = auv_control.pwm_out:main',
        	'control_node = auv_control.control:main',
        ],
    },
)
