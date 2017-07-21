#include "tensorflow/core/framework/op.h"
#include "tensorflow/core/framework/op_kernel.h"
#include "tensorflow/core/framework/shape_inference.h"
#include "tensorflow/core/framework/types.h"
#include "tensorflow/core/lib/core/status.h"
#include "tensorflow/core/platform/stream_executor.h"
#include "tensorflow/stream_executor/cuda/cuda_stream.h"
#include <cuda_runtime_api.h>
#include <cuda_gl_interop.h>
#include <cudaGL.h>
#include <stdio.h>

using namespace tensorflow;

using perftools::gputools::cuda::AsCUDAStreamValue;
using shape_inference::InferenceContext;

extern "C" {
#include "khrplatform.h"
#ifndef EGLAPI
#define EGLAPI KHRONOS_APICALL
#endif

#ifndef EGLAPIENTRY
#define EGLAPIENTRY  KHRONOS_APIENTRY
#endif
#define EGLAPIENTRYP EGLAPIENTRY*

typedef void *EGLContext;
EGLAPI EGLContext EGLAPIENTRY eglGetCurrentContext (void);
#define EGL_NO_CONTEXT                    ((EGLContext)0)
}


REGISTER_OP("ReadGlBuffer")
    .Input("handle: int32")
    .Output("output: uint8")
    .Attr("const_handle: int=0")
    .Attr("width: int")
    .Attr("height: int")
    .Attr("num_images: int=1")
    .SetShapeFn([](InferenceContext* ctx) {
      int width, height, num_images;
      TF_RETURN_IF_ERROR(ctx->GetAttr("width",  &width));
      TF_RETURN_IF_ERROR(ctx->GetAttr("height", &height));
      TF_RETURN_IF_ERROR(ctx->GetAttr("num_images", &num_images));
      ctx->set_output(0, ctx->MakeShape({
        ctx->MakeDim(num_images),
        ctx->MakeDim(height),
        ctx->MakeDim(width),
        ctx->MakeDim(3)  // number of color channels
      }));
      return Status::OK();
    })
    .Doc(R"doc(
Read an OpenGL buffer object
)doc");

class ReadGlBufferOp : public OpKernel {
 public:
  explicit ReadGlBufferOp(OpKernelConstruction* ctx) : OpKernel(ctx) {

    OP_REQUIRES_OK(ctx, ctx->GetAttr("width", &width_ ));
    OP_REQUIRES_OK(ctx, ctx->GetAttr("height", &height_ ));
    OP_REQUIRES_OK(ctx, ctx->GetAttr("num_images", &num_images_ ));
    OP_REQUIRES_OK(ctx, ctx->GetAttr("const_handle", &const_handle_ ));
    is_init_ = false;
    buf_size_ = num_images_ * height_ * width_ * 3;
  }
  void Compute(OpKernelContext* ctx) override {
    printf("XXX Compute\n");
    GLuint handle = (GLuint) const_handle_;
    if (handle == 0) {
        const Tensor* handle_t;
        OP_REQUIRES_OK(ctx, ctx->input("handle", &handle_t));
        OP_REQUIRES(ctx,
            IsLegacyScalar(handle_t->shape()),
            errors::InvalidArgument(
                "handle tensor should be a scalar integer, but got shape ",
                handle_t->shape().DebugString()));
        handle = (GLuint) handle_t->scalar<uint32>()();
    }

    // printf("XXX Create output\n");
    // Tensor* output_t = nullptr;
    // TensorShape output_shape({num_images_, height_, width_, 3});
    // OP_REQUIRES_OK(ctx, ctx->allocate_output(0, output_shape, &output_t));
    // uint8* output_ptr = output_t->flat<uint8>().data();

    printf("XXX pre CUstream\n");
    // CUstream cu_stream = AsCUDAStreamValue(ctx->op_device_context()->stream());
    // int cuda_device;
    // printf("XXX call cudaGetDevice\n");
    // cudaGetDevice(&cuda_device);
    // printf("XXX call cudaGLSetGLDevice %d\n", cuda_device);
    // cudaGLSetGLDevice(cuda_device);
    // printf("XXX call cudaGraphicsGLRegisterBuffer\n");
    // cudaError_t err;
    // if (!is_init_) {
    // err = cudaGraphicsGLRegisterBuffer(
    //     &cuda_gl_resource_,
    //     handle,
    //     cudaGraphicsRegisterFlagsReadOnly);
    // printf("cudaGraphicsGLRegisterBuffer %d\n", err);
    // err = cudaGraphicsMapResources(1, &cuda_gl_resource_, cu_stream);
    // printf("cudaGraphicsMapResources %d\n", err);
    // size_t buf_size;
    // err = cudaGraphicsResourceGetMappedPointer(
    //     (void**) &buf_ptr_,
    //     &buf_size,
    //     cuda_gl_resource_);
    // printf("cudaGraphicsResourceGetMappedPointer %d\n", err);

    if (eglGetCurrentContext() == EGL_NO_CONTEXT) {
      printf("XXX eglGetCurrentContext EGL_NO_CONTEXT\n");
    } else {
      printf("XXX eglGetCurrentContext found!\n");
    }

    CUresult result;
    CUdevice device;
    printf("XXX cuDeviceGet\n");
    result = cuDeviceGet(&device, 0);
    printf("XXX <cuDeviceGet %d\n", result);

    printf("XXX cuGLInit\n");
    result = cuGLInit();
    printf("XXX <cuGLInit %d\n", result);

    CUcontext cu_ctx;
    printf("XXX cuGLCtxCreate\n");
    result = cuGLCtxCreate(&cu_ctx, CU_GL_MAP_RESOURCE_FLAGS_NONE, device);
    printf("XXX <cuGLCtxCreate %d\n", result);

    printf("XXX cuGraphicsGLRegisterBuffer\n");
    result = cuGraphicsGLRegisterBuffer(
      &cuda_gl_resource_, handle, CU_GRAPHICS_REGISTER_FLAGS_NONE);
    printf("XXX <cuGraphicsGLRegisterBuffer %d\n", result);

    // OP_REQUIRES(
    //     ctx,
    //     buf_size_ == buf_size,
    //     errors::Internal("buffer size mismatch ", buf_size, " ", buf_size_));
    is_init_ = true;
    // }

    // err = cudaMemcpyAsync(
    //     output_ptr,
    //     buf_ptr_,
    //     buf_size_,
    //     cudaMemcpyDeviceToDevice,
    //     cu_stream);
    // printf("cudaMemcpyAsync %d\n", err);

    // err = cudaGraphicsUnmapResources(1, &cuda_gl_resource_, cu_stream);
    // printf("cudaGraphicsUnmapResources %d\n", err);
    // err = cudaGraphicsUnregisterResource(cuda_gl_resource_);
    // printf("cudaGraphicsUnregisterResource %d\n", err);
  }

  int   width_;
  int   height_;
  int   num_images_;
  int   buf_size_;
  int  const_handle_;
  bool  is_init_;
  // cudaGraphicsResource_t cuda_gl_resource_;
  CUgraphicsResource cuda_gl_resource_;
  uint8 *buf_ptr_;
};


REGISTER_KERNEL_BUILDER(Name("ReadGlBuffer").Device(DEVICE_GPU).HostMemory("handle"),ReadGlBufferOp);
