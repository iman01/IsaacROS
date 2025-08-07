from setuptools import find_packages
from setuptools import setup

setup(
    name='agrorob_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('agrorob_msgs', 'agrorob_msgs.*')),
)
