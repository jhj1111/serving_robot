from setuptools import find_packages, setup
import os, glob

package_name = 'kiosk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images'), glob.glob('images/*')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhj',
    maintainer_email='jhj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_system_gui = kiosk.order.order_system_gui:main',
            'order_checker = kiosk.order.order_checker:main',
            'kitchen_display_gui = kiosk.kitchen.kitchen_display_gui:main',
            'recommended = kiosk.kitchen.recommended:main',
        ],
    },
)
