from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ontologenius'],
    package_dir={'': 'ontopy'},
    requires=['rospy','ontologenius','ontologenius.srv']
)

setup(**d)
