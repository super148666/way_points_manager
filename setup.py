from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

args = generate_distutils_setup(
    packages=['way_points_manager'],
    package_dir={'': 'src'}
)

setup(**args)