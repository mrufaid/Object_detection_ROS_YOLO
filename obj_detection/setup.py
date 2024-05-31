from setuptools import find_packages, setup

package_name = 'obj_detection'

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
    maintainer='rufaid',
    maintainer_email='rufaid2k21@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'vid_pub=obj_detection.Publisher:main',
        'Vid_sub=obj_detection.Subscriber:main'
        ],
    },
)
