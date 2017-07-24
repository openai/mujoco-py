package(
    default_visibility = ["//tensorflow:internal"],
    features = [
        "-layering_check",
        "-parse_headers",
    ],
)

licenses(["notice"])  # Apache 2.0

load("//tensorflow:tensorflow.bzl", "tf_custom_op_library")

exports_files(["LICENSE"])

tf_custom_op_library(
    name = "cuda_buffer_ops.so",
    srcs = [
        "cuda_buffer_ops.cc",
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