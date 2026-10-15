from setuptools import find_packages, setup

package_name = 'a1_sorting_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sorting_demo.launch.py']),
        ('share/' + package_name + '/config',
            ['config/sorting_params.yaml',
             'config/sorting_params_pipeline.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a1-team',
    maintainer_email='a1-team@example.com',
    description='A1 工件分类搬运:任务状态机、调度、取放执行与结果记录',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_dispatcher_node = a1_sorting_core.task_dispatcher_node:main',
            'pick_place_executor_node = a1_sorting_core.pick_place_executor_node:main',
            'result_recorder_node = a1_sorting_core.result_recorder_node:main',
            'scene_publisher_node = a1_sorting_core.scene_publisher_node:main',
            'run_trials = a1_sorting_core.run_trials:main',
            'voice_command_node = a1_sorting_core.voice_command_node:main',
        ],
    },
)
