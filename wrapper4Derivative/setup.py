# setup.py

import sys

if sys.platform == 'darwin':
    from distutils import sysconfig
    vars = sysconfig.get_config_vars()
    vars['LDSHARED'] = vars['LDSHARED'].replace('-bundle', '-dynamiclib')

from distutils.core import setup, Extension

setup(name="_jacMujoco",
      py_modules=['_jacMujoco'],
      ext_modules=[Extension("_jacMujoco",
                             ["derivative.i","derivative.cpp"],
                             include_dirs=['/Users/michaelmathew/GoogleDrive/mjpro131/include'],
                             library_dirs=['/Users/michaelmathew/GoogleDrive/mjpro131/bin'],
                             libraries=['mujoco131'],
                             swig_opts=['-c++'],
                             )]
      
      )
