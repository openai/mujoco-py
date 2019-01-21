#include "glshim.h"

int usingEGL() {
    return 0;
}

int initOpenGL(int device_id) {
    return 1;
}

int setOpenGLBufferSize(int device_id, int width, int height) {
    return 1;
}

int makeOpenGLContextCurrent(int device_id) {
    return 1;
}

void closeOpenGL() {
}

unsigned int createPBO(int width, int height, int batchSize, int use_short) {
    return 0;
}

void freePBO(unsigned int pixelBuffer) {
}

void copyFBOToPBO(mjrContext* con,
                  unsigned int pbo_rgb, unsigned int pbo_depth,
                  mjrRect viewport, int bufferOffset) {
}

void readPBO(unsigned char *buffer_rgb, unsigned short *buffer_depth,
             unsigned int pbo_rgb, unsigned int pbo_depth,
             int width, int height, int batchSize) {
}
