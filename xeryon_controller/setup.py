from setuptools import find_packages, setup

package_name = 'xeryon_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['xeryon_controller/launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='armanc',
    maintainer_email='armanc@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
            'data_node = xeryon_controller.data_node.data_node:main',
            'linear_node = xeryon_controller.linear_node.linear_node:main',
            'rotary_node = xeryon_controller.rotary_node.rotary_node:main',
        ],
    },
)
