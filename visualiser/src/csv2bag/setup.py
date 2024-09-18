from setuptools import find_packages, setup

package_name = 'csv2bag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amirint',
    maintainer_email='amir.intelli@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = csv2bag.main:main'
        ],
    },
)
