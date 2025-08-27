from setuptools import find_packages, setup

package_name = 'detection_msgs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/labeledDetections.msg']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Custom message with Detection2DArray and label',
    tests_require=['pytest'],
)
