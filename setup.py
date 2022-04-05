from setuptools import setup

package_name = 'neo_nav2_py_costmap2D'

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
    maintainer='pradheep',
    maintainer_email='padhupradheep@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neo_nav2_py_costmap2D = neo_nav2_py_costmap2D.neo_nav2_py_costmap2D:main'
        ],
    },
)
