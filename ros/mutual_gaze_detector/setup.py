from setuptools import setup
import os
from glob import glob

package_name = 'mutual_gaze_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'classifier_weights'), glob(os.path.join('classifier_weights', '*.joblib*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simone Arreghini',
    maintainer_email='simone.arreghini@supsi.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mutual_gaze_detector_node = mutual_gaze_detector.mutual_gaze_detector_node:main',
        ],
    },
)
