from setuptools import find_packages, setup

package_name = 'lab2'

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
    maintainer='anand',
    maintainer_email='ank029@ucsd.edu',
    description='Lab 2 Task 4: Pupper movement based on touch',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtle_sensor = lab2.turtle_sensor:main',
		                    'service = lab2.service_go_pupper:main',
                            'client = lab2.client_go_pupper:main',
                            'gui_service = lab2.gui_pupper_service:main',
                            'gui_client = lab2.gui_pupper_client:main',
                            'gui_client2 = lab2.gui_pupper_client2:main',
                            'gui_client_ssh = lab2.gui_pupper_client_ssh:main',
                            'saveImg = lab2.saveImg:main',
        ],
    },
)

