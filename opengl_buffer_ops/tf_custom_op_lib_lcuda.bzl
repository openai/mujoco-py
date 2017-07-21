
load("//tensorflow:tensorflow.bzl", "tf_custom_op_library_additional_deps")
load("//tensorflow:tensorflow.bzl", "tf_copts")
load("//tensorflow:tensorflow.bzl", "check_deps")
load("@local_config_cuda//cuda:build_defs.bzl", "if_cuda")

def tf_custom_op_lib_lcuda(name, srcs=[], deps=[]):
  cuda_deps = [
      "//tensorflow/core:stream_executor_headers_lib",
      "@local_config_cuda//cuda:cuda_headers",
  ]
  deps = deps + tf_custom_op_library_additional_deps()

  check_deps(name=name+"_check_deps",
             deps=deps + if_cuda(cuda_deps),
             disallowed_deps=["//tensorflow/core:framework",
                              "//tensorflow/core:lib"])

  native.cc_binary(name=name,
                   srcs=srcs,
                   deps=deps + if_cuda(cuda_deps),
                   data=[name + "_check_deps"],
                   copts=tf_copts(),
                   linkshared=1,
                   linkopts = select({
                       "//conditions:default": [
                           "-lm", "-lcuda"
                       ],
                       "//tensorflow:darwin": [ "-lcuda" ],
                   }),
  )

def _cuda_copts():
    common_cuda_opts = ["-x", "cuda", "-DGOOGLE_CUDA=1"]
    return select({
        "//conditions:default": [],
        "@local_config_cuda//cuda:using_nvcc": (
            common_cuda_opts +
            [
                "-nvcc_options=use_fast_math",
            ]
        )
    })


def tf_custom_op_library(name, srcs=[], gpu_srcs=[], deps=[]):
  cuda_deps = [
      "//tensorflow/core:stream_executor_headers_lib",
      "@local_config_cuda//cuda:cudart_static",
      "@local_config_cuda//cuda:cuda_headers",
  ]
  deps = deps + tf_custom_op_library_additional_deps()
  if gpu_srcs:
    basename = name.split(".")[0]
    native.cc_library(
        name = basename + "_gpu",
        srcs = gpu_srcs,
        copts = _cuda_copts(),
        deps = deps + if_cuda(cuda_deps))
    cuda_deps.extend([":" + basename + "_gpu"])

  check_deps(name=name+"_check_deps",
             deps=deps + if_cuda(cuda_deps),
             disallowed_deps=["//tensorflow/core:framework",
                              "//tensorflow/core:lib"])

  native.cc_binary(name=name,
                   srcs=srcs,
                   deps=deps + if_cuda(cuda_deps),
                   data=[name + "_check_deps"],
                   copts=tf_copts(),
                   linkshared=1,
                   linkopts = select({
                       "//conditions:default": [
                           "-lm", "-l:libEGL.so.1", "-L/usr/local/nvidia/lib64"
                       ],
                       "//tensorflow:darwin": [ "-lcuda" ],
                   }),
  )