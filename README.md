# MuJoCo Python Bindings

[![Build Status](https://travis-ci.org/openai/mujoco-py.svg?branch=master)](https://travis-ci.org/openai/mujoco-py)

MuJoCo is a physics engine which can do very detailed efficient
simulations with contacts. This library lets you use MuJoCo from
Python.

Note that MuJoCo tends to change significantly between versions, so
this library is likely to stay pinned to 1.31 for the near future.

# Installing this library

You can install this library using:

```
pip install mujoco-py
```

MuJoCo isn't open-source, so you'll also need to set download the
MuJoCo binaries and obtain license key.

## Obtaining the binaries and license key

1. Obtain a 30-day free trial on the MuJoCo website:
   https://www.roboti.us/trybuy.html. The license key will arrive in
   an email with your username and password.
2. Download the MuJoCo version 1.31 binaries for
   [Linux](https://www.roboti.us/active/mjpro131_linux.zip),
   [OSX](https://www.roboti.us/active/mjpro131_osx.zip), or
   [Windows](https://www.roboti.us/active/mjpro131_windows.zip).
3. Download your license key (the `mjkey.txt` file from your email)
   and unzip the downloaded mjpro bundle.
4. Place these in `~/.mujoco/mjkey.txt` and `~/.mujoco/mjpro131`. You
   can alternatively set the following environment variables:

```
export MUJOCO_PY_MJKEY_PATH=/path/to/mjkey.txt
export MUJOCO_PY_MJPRO_PATH=/path/to/mjpro131
```

## Testing

Run:

```
make test
```
