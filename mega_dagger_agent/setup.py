from setuptools import setup

package_name = 'mega_dagger_agent'

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
    description='MEGA-DAgger Agent',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = mega_dagger_agent.agent_node:main',
        ],
    },
)
