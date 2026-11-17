from setuptools import find_packages, setup

package_name = 'phoenyx_i_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ir-26-t12',
    maintainer_email='cyprianek2005@gmail.com',
    description='Autonomous control nodes for the Phoenyx-I Mars mission',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "corridor_controller = phoenyx_i_control.corridor_controller:main",
        ],
    },
)
