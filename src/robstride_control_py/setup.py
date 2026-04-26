from setuptools import find_packages, setup

package_name = 'robstride_control_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robstride_py.launch.py']),
        ('share/' + package_name + '/config', ['config/robstride.yaml']),
    ],
    install_requires=['setuptools', 'python-can>=4.0.0', 'numpy>=1.21.0', 'tqdm>=4.62.0'],
    zip_safe=True,
    maintainer='Metabot maintainers',
    maintainer_email='metabot@example.com',
    description='ROS 2 Jazzy Python driver for RobStride MIT-mode motors '
                '(adapted from Seeed-Projects/RobStride_Control).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robstride_py_node = robstride_control_py.robstride_py_node:main',
        ],
    },
)
