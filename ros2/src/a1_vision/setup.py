from setuptools import find_packages, setup

package_name = 'a1_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vision_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a1-team',
    maintainer_email='a1-team@example.com',
    description='A1 工件分类搬运:相机颜色识别',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_classifier_node = a1_vision.vision_classifier_node:main',
        ],
    },
)
