from setuptools import setup

package_name = 'hyper_control'

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
    maintainer='lwx',
    maintainer_email='3065746398@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'talker = hyper_control.joint_publisher:main',
             "monte_karlo = hyper_control.monte_karlo:main",
             "swift_sim = hyper_control.hyper_robot:main"
        ],
    },
)
