SHELL := /bin/bash
.PHONY: all clean build test mount_shell shell upload check-env

VERSION := `cd mujoco_py; python -c "from version import get_version;print(get_version())"; cd ..`
PYTHON_VERSION := "36"
DOCKER_NAME := quay.io/openai/mujoco_py:$(USER)_$(VERSION)
DOCKER := $(shell type -p nvidia-docker || echo docker)
UUID := $(shell uuidgen)

all: test

clean:
	rm -rf mujoco_py.egg-info
	rm -rf */__pycache__
	rm -rf */*/__pycache__
	rm -rf mujoco_py/generated/_pyxbld*
	rm -rf mujoco_py/generated/*.so
	rm -rf mujoco_py/generated/*.dll
	rm -rf mujoco_py/generated_cymj*
	rm -rf mujoco_py/cythonlock_*.pyc
	rm -rf mujoco_py/cymj.c
	rm -rf mujoco_py/__pycache__
	rm -rf dist
	rm -rf build

build:
	docker build -t $(DOCKER_NAME) .

push:
	docker push $(DOCKER_NAME)

pull:
	docker pull $(DOCKER_NAME)

test: build
	# run it interactive mode so we can abort with CTRL+C
	$(DOCKER) run --rm -i $(DOCKER_NAME) pytest

test_gpu: build push
	$(eval NODE="$(shell cirrascale-cli reserve -g 1080ti -t 10m)")
	$(eval GPUS="$(shell echo $(NODE)| grep -oE '[^:]+f' | cut -c1-1 )")
	$(eval NODE="$(shell echo $(NODE)| grep -oE '[^=]+:' | sed 's/://')")
	ssh -t -o StrictHostKeyChecking=no $(NODE) 'docker pull $(DOCKER_NAME) && export GPUS=$(GPUS) && nvidia-docker run --rm -e GPUS -i $(DOCKER_NAME) pytest -m "not requires_glfw"'

shell:
	$(DOCKER) run --rm -it $(DOCKER_NAME) /bin/bash

mount:
	$(DOCKER) run -e GPUS -it -v `pwd`:/mujoco_py-dev $(DOCKER_NAME) /bin/bash -c "pip uninstall -y mujoco-py; rm -rf /mujoco_py; (cd /mujoco_py-dev; export PYTHONPATH='/mujoco_py-dev'; /bin/bash)"

cirra:
	$(eval NODE="$(shell cirrascale-cli reserve)")
	$(eval GPUS="$(shell echo $(NODE)| grep -oE '[^:]+f' | cut -c1-1 )")
	$(eval NODE="$(shell echo $(NODE)| grep -oE '[^=]+:' | sed 's/://')")
	tmux new-session  "while :; do rsync -e 'ssh -o StrictHostKeyChecking=no' --delete -rzai --out-format='%t %f %b' --chmod=Du+rwx --exclude='*extensionbuilder.so' --exclude='dist' --exclude='cymj.c' --exclude='_pyxbld_*' --exclude='__pycache__' --exclude='*.egg-info' --exclude='.cache' --exclude='.git' --exclude='*.pyc' --exclude='*.swp' --exclude='.idea' . $(NODE):~/mujoco_py/ ; sleep 1; done" \; \
	     split-window "ssh -t -o StrictHostKeyChecking=no $(NODE) 'mkdir -p ~/mujoco_py && cd ~/mujoco_py && export GPUS=$(GPUS) && /bin/bash'; " \; select-layout 9ce0,204x51,0,0[204x4,0,0,32,204x46,0,5,33]

# Requires to generate all *.so files. Call make generate_mac_so on mac; 
# Call make generate_cpu_so on mac or linux. 
# 
# Call make generate_gpu_so on linux with nvidia-docker. The easiest way is to get to cirrascale, and call: make cirra. 
# Then you have to copy back generated cymj_linuxgpuextensionbuilder.so file.
#
# Gather all *.so files.
upload:
    # Commented out for now, binary distributions seem not to work very well
	# test -f ./mujoco_py/generated/cymj_$(VERSION)_$(PYTHON_VERSION)_linuxcpuextensionbuilder.so || exit -1
	# test -f ./mujoco_py/generated/cymj_$(VERSION)_$(PYTHON_VERSION)_linuxgpuextensionbuilder.so || exit -1
	rm -rf dist
	python setup.py sdist
	twine upload dist/*

generate_gpu_so:
	rm -f ./mujoco_py/generated/cymj_*_linuxgpuextensionbuilder.so
	nvidia-docker run -it --name $(UUID) $(DOCKER_NAME) bash -c "make clean;python -c 'import mujoco_py'"
	nvidia-docker cp $(UUID):/mujoco_py/mujoco_py/generated/cymj_$(VERSION)_$(PYTHON_VERSION)_linuxgpuextensionbuilder.so mujoco_py/generated/

generate_cpu_so:
	rm -f ./mujoco_py/generated/cymj_*_linuxcpuextensionbuilder.so
	docker run -it --name $(UUID) $(DOCKER_NAME) bash -c "make clean;python -c 'import mujoco_py'"
	docker cp $(UUID):/mujoco_py/mujoco_py/generated/cymj_$(VERSION)_$(PYTHON_VERSION)_linuxcpuextensionbuilder.so mujoco_py/generated/

