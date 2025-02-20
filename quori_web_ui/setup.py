from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'quori_web_ui'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name , "launch"), glob('launch/*launch.py')),
        (os.path.join('share', package_name , "launch"), glob('launch/*launch.xml')),
        (os.path.join('share', package_name , "html"), glob('html/*.html')),
        (os.path.join('share', package_name , "html/css"), glob('html/css/*')),
        (os.path.join('share', package_name , "html/images"), glob('html/images/*')),
        (os.path.join('share', package_name , "html/scripts"), glob('html/scripts/*.js')),
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ralph Gnauck',
    maintainer_email='ralph@ologicinc.com',
    description='Quori Web UI',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quori_web_ui = quori_web_ui.quori_web_ui:main'
        ],
    },
)
