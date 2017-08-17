SHELL := /bin/bash
.PHONY: all clean build test mount_shell shell upload check-env

MUJOCO_LICENSE_PATH ?= ~/.mujoco/mjkey.txt
DOCKER_NAME := quay.io/openai/mujoco_py:$(USER)
DOCKER := $(shell type -p nvidia-docker || echo docker)

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

build: check-license
	cp $(MUJOCO_LICENSE_PATH) mjkey.txt
	docker build -t $(DOCKER_NAME) . || rm mjkey.txt && rm mjkey.txt

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
	tmux new-session  "while :; do rsync -e 'ssh -o StrictHostKeyChecking=no' --delete -rzai --out-format='%t %f %b' --chmod=Du+rwx --exclude='dist' --exclude='cymj.c' --exclude='_pyxbld_*' --exclude='*extensionbuilder.so' --exclude='__pycache__' --exclude='*.egg-info' --exclude='.cache' --exclude='.git' --exclude='*.pyc' --exclude='*.swp' --exclude='.idea' . $(NODE):~/mujoco_py/ ; sleep 1; done" \; \
	     split-window "ssh -t -o StrictHostKeyChecking=no $(NODE) 'mkdir -p ~/mujoco_py && cd ~/mujoco_py && export GPUS=$(GPUS) && /bin/bash'; " \; select-layout 9ce0,204x51,0,0[204x4,0,0,32,204x46,0,5,33]

upload:
	rm -rf dist
	python setup.py sdist
	twine upload dist/*

check-license:
ifeq ("","$(wildcard $(MUJOCO_LICENSE_PATH))")
    $(error "License key not found at location $(MUJOCO_LICENSE_PATH). Use MUJOCO_LICENSE_PATH to specify its path")
endif
