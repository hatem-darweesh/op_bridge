from __future__ absolute import
import sys
__future_module__ = True

if sys.version_info[0] < 3:
    from Queue import *
else: 
    raise ImportError('This package shoud not be accessible on Python 3.')
