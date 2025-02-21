from setuptools import find_packages, setup

package_name = 'user_func_implement'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'user_func_implement': ['sqlite/*']},  #传送icon文件夹到install
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st',
    maintainer_email='st@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_func_main = user_func_implement.user_func_main:main'
        ],
    },
)
