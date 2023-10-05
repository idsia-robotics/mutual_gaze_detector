from setuptools import setup
import os
from glob import glob

package_name = 'users_landmarks_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, os.path.join(package_name,'FaceLandmarksExtractor')],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.task'))),
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
            'face_landmarks_node = users_landmarks_tracking.face_landmarks_node:main',
        ],
    },
)
