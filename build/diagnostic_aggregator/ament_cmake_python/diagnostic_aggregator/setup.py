from setuptools import find_packages
from setuptools import setup

setup(
    name='diagnostic_aggregator',
    version='4.3.1',
    packages=find_packages(
        include=('diagnostic_aggregator', 'diagnostic_aggregator.*')),
)
