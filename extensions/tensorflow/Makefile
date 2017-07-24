.PHONY: build

TENSORFLOW_PATH ?= /tensorflow
LIBRARY_NAME = cuda_buffer_ops


build:
	cp *.cc $(TENSORFLOW_PATH)/tensorflow/core/user_ops
	cp tf_custom_op_lib_lcuda.bzl $(TENSORFLOW_PATH)/tensorflow/core/user_ops
	cp BUILD $(TENSORFLOW_PATH)/tensorflow/core/user_ops
	rm -f $(TENSORFLOW_PATH)/bazel-bin/tensorflow/core/user_ops/$(LIBRARY_NAME).so
	cd $(TENSORFLOW_PATH); bazel build -c opt --config=cuda //tensorflow/core/user_ops:$(LIBRARY_NAME).so --cxxopt="-D_GLIBCXX_USE_CXX11_ABI=0" -s
	cp -f $(TENSORFLOW_PATH)/bazel-bin/tensorflow/core/user_ops/$(LIBRARY_NAME).so $(LIBRARY_NAME)

