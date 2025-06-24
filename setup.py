from setuptools import find_packages, setup

package_name = 'ultrasonic_a02yyuw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ultrasonic_a02yyuw.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/ultrasonic_a02yyuw_description.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarkar',
    maintainer_email='sarkar@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_a02yyuw = ultrasonic_a02yyuw.ultrasonic_a02yyuw:main'
        ],
    },
)
