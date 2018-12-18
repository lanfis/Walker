#!/ust/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

tf_nn = generate_distutils_setup(
    packages=['/../../../include/tf_nn'],
    package_dir={'': 'src'},
    )

setup(**tf_nn)
