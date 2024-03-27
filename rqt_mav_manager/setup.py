from setuptools import find_packages, setup

package_name = 'rqt_mav_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/MavManager.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pranav Shah',
    maintainer_email='pranavpshah2098@gmail.com',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rqt_mav_manager = rqt_mav_manager.rqt_mav_manager:main"
        ],
    },
)
