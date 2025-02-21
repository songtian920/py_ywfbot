from setuptools import find_packages, setup

package_name = 'ui_pack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'ui_pack': ['qml/svg_icons/*','qml/*']},  #传送icon文件夹到install
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('lib/python3.10/site-packages/' + package_name, [package_name+'/qml_main.qml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st',
    maintainer_email='userEmail',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_main = ui_pack.ui_main:main'
        ],
    },
)
