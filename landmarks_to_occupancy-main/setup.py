from setuptools import setup

package_name = 'landmarks_to_occupancy'

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
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Convert VSLAM landmarks to occupancy grid',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmarks_to_occupancy = landmarks_to_occupancy.landmarks_to_occupancy:main'
        ],
    },
)

