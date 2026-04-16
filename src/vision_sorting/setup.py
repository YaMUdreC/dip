from setuptools import find_packages, setup

package_name = 'vision_sorting'

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
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Vision sorting module for manipulator',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_node = vision_sorting.test_node:main',
            'image_publisher = vision_sorting.image_publisher:main',
            'image_subscriber = vision_sorting.image_subscriber:main',
            'vision_node = vision_sorting.vision_node:main',
            'detection_listener = vision_sorting.detection_listener:main',
            'target_selector = vision_sorting.target_selector:main',
            'target_3d_estimator = vision_sorting.target_3d_estimator:main',
        ],
    },
)
