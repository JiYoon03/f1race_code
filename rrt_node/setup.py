from setuptools import setup

package_name = 'rrt_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neel Shejwalkar',
    maintainer_email='nshej@seas.upenn.edu',
    description='RRT* + Pure Pursuit hybrid racing node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rrt_node = rrt_node.rrt_node:main',
        ],
    },
)
