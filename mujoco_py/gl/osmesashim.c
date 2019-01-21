#include <GL/osmesa.h>
#include "glshim.h"

OSMesaContext ctx;

// this size was picked pretty arbitrarily
int BUFFER_WIDTH = 1024;
int BUFFER_HEIGHT = 1024;
// 4 channels for RGBA
unsigned char buffer[1024 * 1024 * 4];

int is_initialized = 0;

int usingEGL() {
    return 0;
}

int initOpenGL(int device_id) {
    if (is_initialized)
        return 1;

    // note: device id not used here
    ctx = OSMesaCreateContextExt(GL_RGBA, 24, 8, 8, 0);
    if( !ctx ) {
        printf("OSMesa context creation failed\n");
        return -1;
    }

    if( !OSMesaMakeCurrent(ctx, buffer, GL_UNSIGNED_BYTE, BUFFER_WIDTH, BUFFER_HEIGHT) ) {
        printf("OSMesa make current failed\n");
        return -1;
    }

    is_initialized = 1;

    return 1;
}

int makeOpenGLContextCurrent(int device_id) {
    // Don't need to make context current here, causes issues with large tests
    return 1;
}

int setOpenGLBufferSize(int device_id, int width, int height) {
    if (width > BUFFER_WIDTH || height > BUFFER_HEIGHT) {
        printf("Buffer size too big\n");
        return -1;
    }
    // Noop since we don't support changing the actual buffer
    return 1;
}

void closeOpenGL() {
    if (is_initialized) {
        OSMesaDestroyContext(ctx);
        is_initialized = 0;
    }
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
