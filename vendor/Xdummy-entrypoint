#!/usr/bin/python
import argparse
import os
import sys
import subprocess

parser = argparse.ArgumentParser()
args, extra_args = parser.parse_known_args()
subprocess.Popen(["nohup", "Xdummy"], stdout=open('/dev/null', 'w'), stderr=open('/dev/null', 'w'))
os.environ['DISPLAY'] = ':0'
if not extra_args:
    sys.argv = ['/bin/bash']
else:
    sys.argv = extra_args
# Explicitly flush right before the exec since otherwise things might get
# lost in Python's buffers around stdout/stderr (!).
sys.stdout.flush()
sys.stderr.flush()
os.execvpe(sys.argv[0], sys.argv, os.environ)

