from setuptools import setup

package_name = 'lqr'

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
    maintainer='Derek Zhou',
    maintainer_email='derekzhou1105@gmail.com',
    description='Kinematic Lateral LQR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lqr_node = lqr.lqr_node:main',
        ],
    },
)
