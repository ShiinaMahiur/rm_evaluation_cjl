from setuptools import setup

package_name = 'rm_vision_probe'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='chen@example.com',
    description='Minimal probes for Daedalus S0',
    license='MIT',
    entry_points={
        'console_scripts': [
            'image_probe = rm_vision_probe.image_probe:main',
            'detector_traditional = rm_vision_probe.detector_traditional:main',
            'gimbal_cmd = rm_vision_probe.gimbal_cmd:main',
        ],
    },
)
