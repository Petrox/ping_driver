# DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD. 
# IF YOU MANUALLY INVOKE, IT CAN BREAK YOUR ROS INSTALL

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
# This parenthesis literally HAS to be on this line, f*ck you python for making me spend an hour debugging that 
setup_args = generate_distutils_setup (
    packages=['ping_driver', 'ping_driver.cfg', 'ping_driver.msg', 'ping_driver.packages'],
    scripts=['scripts/pingDriver.py'],
    package_dir={'': 'src'}
)

setup(**setup_args)