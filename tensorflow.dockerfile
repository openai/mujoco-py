FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu16.04

RUN apt-get update -q && apt-get install -y \
    curl \
    git \
    libcupti-dev \
     mesa-common-dev \
    openjdk-8-jdk \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-wheel \
    unzip \
    vim \
    wget \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Bazel
RUN wget https://github.com/bazelbuild/bazel/releases/download/0.5.2/bazel-0.5.2-installer-linux-x86_64.sh
RUN chmod +x bazel-0.5.2-installer-linux-x86_64.sh
RUN ./bazel-0.5.2-installer-linux-x86_64.sh --user
ENV PATH=/root/bin:$PATH

# Fix to make Python easier to work with
RUN ln -s /usr/bin/pip3 /usr/bin/pip
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN pip install --upgrade pip

# Fix to make it easier to set CUDNN path
RUN cp -P /usr/include/cudnn.h /usr/local/cuda/include
RUN cp -P /usr/lib/x86_64-linux-gnu/libcudnn* /usr/local/cuda/lib64
RUN chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*

# Prepare CUDA for Tensorflow
ENV PATH=/usr/local/cuda/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64:$LD_LIBRARY_PATH
ENV CUDA_HOME=/usr/local/cuda

# Clone Tensorflow
RUN git clone https://github.com/tensorflow/tensorflow /tensorflow
WORKDIR /tensorflow
RUN git checkout r1.2

# Set up all constants for Tensorflow's configure script
ENV TF_NEED_CUDA=1
ENV TF_NEED_GCP=0
ENV TF_NEED_HDFS=0
ENV TF_NEED_OPENCL=0
ENV TF_CUDA_CLANG=0
ENV TF_CUDA_VERSION=8.0
ENV TF_CUDA_COMPUTE_CAPABILITIES="3.5,6.0,6.1"
ENV GCC_HOST_COMPILER_PATH="/usr/bin/gcc"
ENV CUDA_TOOLKIT_PATH="/usr/local/cuda"
ENV CUDNN_INSTALL_PATH="/usr/local/cuda"
ENV PYTHON_BIN_PATH=/usr/bin/python
ENV PYTHON_LIB_PATH=/usr/lib/python3/dist-packages
ENV TF_NEED_JEMALLOC=1
ENV TF_NEED_MKL=0
ENV TF_NEED_GCP=0
ENV TF_ENABLE_XLA=0
ENV TF_NEED_VERBS=0
ENV TF_CUDNN_VERSION=5.1.10
ENV CC_OPT_FLAGS="-march=native"

# Prepare the compilation
RUN ./configure

# Run compilation
RUN bazel build --config=opt \
    --config=cuda //tensorflow/tools/pip_package:build_pip_package \
    --cxxopt="-D_GLIBCXX_USE_CXX11_ABI=0"

# Prepare and install wheel
RUN bazel-bin/tensorflow/tools/pip_package/build_pip_package /tmp/tensorflow_pkg
RUN pip install /tmp/tensorflow_pkg/tensorflow-1.2.1-cp35-cp35m-linux_x86_64.whl
WORKDIR /root

# Validate by running the following in Python:
# import tensorflow as tf
# hello = tf.constant('Hello, TensorFlow!')
# sess = tf.Session()
# print(sess.run(hello))
