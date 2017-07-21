package(
    default_visibility = ["//tensorflow:internal"],
    features = [
        "-layering_check",
        "-parse_headers",
    ],
)

licenses(["notice"])  # Apache 2.0

#load("//tensorflow:tensorflow.bzl", "tf_cuda_tests_tags")
#load("//tensorflow:tensorflow.bzl", "tf_custom_op_library")
load("//tensorflow:core/user_ops/tf_custom_op_lib_lcuda.bzl", "tf_custom_op_library")

exports_files(["LICENSE"])

tf_custom_op_library(
    name = "opengl_buffer_ops.so",
    srcs = [
        "read_gl_buffer_op.cc",
        "egl.h",
        "eglext.h",
        "eglplatform.h",
        "khrplatform.h",
    ],
    gpu_srcs = [
    ]
)


filegroup(
    name = "all_files",
    srcs = glob(
        ["**/*"],
        exclude = [
            "**/METADATA",
            "**/OWNERS",
        ],
    ),
    visibility = ["//tensorflow:__subpackages__"],
)