from setuptools import setup

package_name = 'tipsy_turtle'

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
    maintainer='syqua',
    maintainer_email='syqua@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = tipsy_turtle.my_first_node:main",
            "draw_circle = tipsy_turtle.sim_circle:main",
            "pose_sub = tipsy_turtle.pose_subscriber:main",
            "main = tipsy_turtle.turtle_controller:main"
        ],
    },
)
