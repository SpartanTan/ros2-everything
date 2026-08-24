from setuptools import setup

package_name = 'sam_bot_description'

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
    maintainer='zhicun',
    maintainer_email='tanzc9866@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
                'talker = sam_bot_description.publisher_member_function:main',
                'teleop_twist = sam_bot_description.teleop_twist:main',
        ],
    },
)
