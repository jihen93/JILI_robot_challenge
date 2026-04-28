from setuptools import find_packages, setup

package_name = 'my_project_package'

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
    maintainer='turtle',
    maintainer_email='turtle@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_robot = my_project_package.challenge1:main',
	    'run_challenge2 = my_project_package.challenge2:main',
            'run_challenge5 = my_project_package.challenge5:main',
        ],
    },
)
