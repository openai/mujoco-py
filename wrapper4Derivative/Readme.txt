The file derivative.cpp is from Mujoco. It is given in the official web page as an example. This code belongs to RobotiLLC.
Refer: http://www.mujoco.org/book/pro.html#saDerivative
This is a wrapper for the same cpp function added to the mujoco_py version.

Usage: to get the jacobian of a model use it with the following usage:

J = model.cmptJac() where model is mjmodel in mujoco_py

J is a 6*model.nv*model*nv size array. There are a total of 6 Jacobians. Each is a square matrix with dimension = model.nv

Order of Jacobians :

1) dinv/dpos
2) dinv/dvel
3) dinv/dacc
4) dacc/dpos
5) dacc/dvel
6) dacc/dfrc
