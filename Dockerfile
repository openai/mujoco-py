FROM ubuntu:14.04

RUN apt-get update && apt-get install -y \
    python2.7 \
    python2.7-dev \
    python-pip \
    unzip \
    xorg-dev \
    libgl1-mesa-dev \
    xvfb \
    libxinerama1 \
    libxcursor1 \
    libglu1-mesa \
    cmake \
    gfortran \
    libblas-dev \
    libatlas-dev \
    liblapack-dev \
    libjpeg62 \
    libjpeg62-dev \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /mujoco-py/requirements.txt
RUN pip install -r /mujoco-py/requirements.txt

COPY . /mujoco-py

ENV PYTHONPATH=/mujoco-py
WORKDIR /mujoco-py
ENTRYPOINT ["bin/docker-entrypoint.sh"]
CMD ["nosetests", "tests/"]
