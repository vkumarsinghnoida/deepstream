from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'demon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='84572128+vkumarsinghnoida@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = demon.talker:main',
            'listener = demon.listener:main',
            'input = demon.input:main',
        ],
    },
)
