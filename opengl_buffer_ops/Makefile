.PHONY: build

TENSORFLOW_PATH ?= /tensorflow
LIBRARY_NAME = opengl_buffer_ops


build:
	cp *.cc $(TENSORFLOW_PATH)/tensorflow/core/user_ops
	cp BUILD $(TENSORFLOW_PATH)/tensorflow/core/user_ops
	rm $(TENSORFLOW_PATH)/bazel-bin/tensorflow/core/user_ops/$(LIBRARY_NAME).so
	cd $(TENSORFLOW_PATH); bazel build -c opt --config=cuda //tensorflow/core/user_ops:$(LIBRARY_NAME).so --cxxopt="-D_GLIBCXX_USE_CXX11_ABI=0"
	cp -f $(TENSORFLOW_PATH)/bazel-bin/tensorflow/core/user_ops/$(LIBRARY_NAME).so $(LIBRARY_NAME)

