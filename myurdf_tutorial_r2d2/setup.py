import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'myurdf_tutorial_r2d2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhicun',
    maintainer_email='tanzc9866@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = myurdf_tutorial_r2d2.state_publisher:main'
        ],
    },
)
