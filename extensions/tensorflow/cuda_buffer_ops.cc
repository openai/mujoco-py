#include "tensorflow/core/framework/op.h"
#include "tensorflow/core/framework/op_kernel.h"
#include "tensorflow/core/framework/shape_inference.h"
#include "tensorflow/core/framework/types.h"
#include "tensorflow/core/lib/core/status.h"
#include "tensorflow/core/platform/stream_executor.h"
#include "tensorflow/stream_executor/cuda/cuda_stream.h"
#include <cuda_runtime_api.h>

using namespace tensorflow;

using perftools::gputools::cuda::AsCUDAStreamValue;
using shape_inference::InferenceContext;


REGISTER_OP("ReadCudaBufferUint8")
    .Output("output: uint8")
    .Attr("buffer_pointer: int")
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
Create a new tensor from the contents of a uint8 CUDA Buffer
representing a batch of 3-channel images.
)doc");

class ReadCudaBufferUint8Op : public OpKernel {
public:
    explicit ReadCudaBufferUint8Op(OpKernelConstruction* ctx) : OpKernel(ctx) {
        int64 buf_ptr;
        OP_REQUIRES_OK(ctx, ctx->GetAttr("buffer_pointer", &buf_ptr));
        buf_ptr_ = (void*) buf_ptr;

        OP_REQUIRES_OK(ctx, ctx->GetAttr("width", &width_ ));
        OP_REQUIRES_OK(ctx, ctx->GetAttr("height", &height_ ));
        OP_REQUIRES_OK(ctx, ctx->GetAttr("num_images", &num_images_ ));
        buf_size_ = num_images_ * height_ * width_ * 3;
    }

    void Compute(OpKernelContext* ctx) override {
        Tensor* output_t = nullptr;
        TensorShape output_shape({num_images_, height_, width_, 3});
        OP_REQUIRES_OK(ctx, ctx->allocate_output(0, output_shape, &output_t));
        uint8* output_ptr = output_t->flat<uint8>().data();

        CUstream cu_stream = AsCUDAStreamValue(ctx->op_device_context()->stream());
        cudaError_t err;
        err = cudaMemcpyAsync(
            output_ptr,
            buf_ptr_,
            buf_size_,
            cudaMemcpyDeviceToDevice,
            cu_stream);
        OP_REQUIRES(ctx,
                    err == cudaSuccess,
                    errors::Internal("Failed to copy CUDA buffer ", err));
    }

private:
    int   width_;
    int   height_;
    int   num_images_;
    int   buf_size_;
    void *buf_ptr_;
};


REGISTER_KERNEL_BUILDER(
    Name("ReadCudaBufferUint8").Device(DEVICE_GPU),
    ReadCudaBufferUint8Op);

REGISTER_OP("ReadCudaBufferFloat")
    .Output("output: float")
    .Attr("buffer_pointer: int")
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
            ctx->MakeDim(width)
        }));
        return Status::OK();
    })
    .Doc(R"doc(
Create a new tensor from the contents of a float CUDA Buffer
representing a batch of depth images.
)doc");

class ReadCudaBufferFloatOp : public OpKernel {
public:
    explicit ReadCudaBufferFloatOp(OpKernelConstruction* ctx) : OpKernel(ctx) {
        int64 buf_ptr;
        OP_REQUIRES_OK(ctx, ctx->GetAttr("buffer_pointer", &buf_ptr));
        buf_ptr_ = (void*) buf_ptr;

        OP_REQUIRES_OK(ctx, ctx->GetAttr("width", &width_ ));
        OP_REQUIRES_OK(ctx, ctx->GetAttr("height", &height_ ));
        OP_REQUIRES_OK(ctx, ctx->GetAttr("num_images", &num_images_ ));
        buf_size_ = num_images_ * height_ * width_ * sizeof(float);
    }

    void Compute(OpKernelContext* ctx) override {
        Tensor* output_t = nullptr;
        TensorShape output_shape({num_images_, height_, width_});
        OP_REQUIRES_OK(ctx, ctx->allocate_output(0, output_shape, &output_t));
        float* output_ptr = output_t->flat<float>().data();

        CUstream cu_stream = AsCUDAStreamValue(ctx->op_device_context()->stream());
        cudaError_t err;
        err = cudaMemcpyAsync(
            output_ptr,
            buf_ptr_,
            buf_size_,
            cudaMemcpyDeviceToDevice,
            cu_stream);
        OP_REQUIRES(ctx,
                    err == cudaSuccess,
                    errors::Internal("Failed to copy CUDA buffer ", err));
    }

private:
    int   width_;
    int   height_;
    int   num_images_;
    int   buf_size_;
    void *buf_ptr_;
};


REGISTER_KERNEL_BUILDER(
    Name("ReadCudaBufferFloat").Device(DEVICE_GPU),
    ReadCudaBufferFloatOp);