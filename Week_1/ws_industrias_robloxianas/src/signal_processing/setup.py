from setuptools import find_packages, setup

package_name = 'signal_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/challenge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reiv',
    maintainer_email='reiv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'signal_gen = signal_processing.signal_gen:main',
            'signal_proc = signal_processing.signal_proc:main'
        ],
    },
)
