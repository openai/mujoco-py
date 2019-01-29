# Windows Build Instructions for [mujoco-py](https://openai.github.io/mujoco-py/build/html/index.html)

To get MuJoCo Python working on windows with a Conda environment, perform the 
following setup:

 - Download [MuJoCo 1.5](https://www.roboti.us/download/mjpro150_win64.zip) 
 and extract it to `%userprofile%\.mujoco\mjpro150`.
 
 - Place your key file at `%userprofile%\.mujoco\mjkey.txt` and
 `%userprofile%\.mujoco\mjpro150\bin\mjkey.txt`
 
 - Add `%userprofile%\.mujoco\mjpro150\bin` to the PATH
 
 - Clone [mujoco-py](https://github.com/openai/mujoco-py) somewhere, e.g.
 `C:\Development\mujoco-py`
 - Install
 [Microsoft Visual C++ Build Tools 2015](https://download.microsoft.com/download/5/f/7/5f7acaeb-8363-451f-9425-68a90f98b238/visualcppbuildtools_full.exe?fixForIE=.exe)
 , checking the options for
 `Windows 8.1 SDK` and `Windows 10 SDK`

Then, use the following build steps:

 - Open `Visual C++ 2015 x64 Native Build Tools Command Prompt` from the 
 Start Menu or `C:\Program Files (x86)\Microsoft Visual C++ Build Tools`
 
 - Change to the folder where you checked out the project, e.g. `cd 
 C:\Development\mujoco-py`
 - Activate desired python Conda Env: `activate python36`
 - Update setuptools: `python -m pip install --upgrade setuptools`
 - Install requirements: `pip install -r requirements.txt` and dev requirements
 `pip install -r requirements.dev.txt`
 - Edit `C:\Development\mujoco-py\scripts\gen_wrappers.py` and 
 `C:\Development\mujoco-py\mujoco_py\generated\wrappers.pxi` and replace all 
 instances of
 
   `isinstance(addr, (int, np.int32, np.int64))`
   
   with
   
   `hasattr(addr, '__int__')`
 
 - From the folder `C:\Development\mujoco-py`, import mujoco_py once to force
   compilation with the updated files: `python -c "import mujoco_py"`
   
 - Install mujoco-py to the conda env: `python setup.py install`

You should now be able to import and use mujoco_py from any python script 
running in that environment.
