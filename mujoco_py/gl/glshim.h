#ifndef __GLSHIM_H__
#define __GLSHIM_H__

#include "mujoco.h"
#include "mjrender.h"

#ifdef __cplusplus
extern "C" {
#endif

int initOpenGL(int device_id);
void closeOpenGL();
int makeOpenGLContextCurrent(int device_id);
int setOpenGLBufferSize(int device_id, int width, int height);
#ifdef __cplusplus
}  // extern "C"
#endif

#endif
