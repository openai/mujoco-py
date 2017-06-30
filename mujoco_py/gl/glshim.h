#ifndef __GLSHIM_H__
#define __GLSHIM_H__

int initOpenGL(int device_id);
void closeOpenGL();
int makeOpenGLContextCurrent(int device_id);
int setOpenGLBufferSize(int device_id, int width, int height);

#endif
