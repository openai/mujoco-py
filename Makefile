.PHONY: all clean build test mount_shell shell upload check-env

MUJOCO_LICENSE_PATH ?= ~/.mujoco/mjkey.txt
DOCKER_NAME := quay.io/openai/gpr:$(USER)

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
	docker run --rm -i $(DOCKER_NAME) pytest

shell:
	docker run --rm -it $(DOCKER_NAME) /bin/bash

mount:
	NV_GPU=$(GPUS) $(DOCKER) run -it -v `pwd`:/gpr-dev $(DOCKER_NAME) /bin/bash -c "pip uninstall -y mujoco-py; rm -rf /mujoco_py; (cd /mujoco_py-dev; export PYTHONPATH='/mujoco_py-dev'; /bin/bash)"

cirra:
	$(eval NODE="$(shell cirrascale-cli reserve)")
	$(eval GPUS="$(shell echo $(NODE)| grep -oE '[^:]+f' | cut -c1-1 )")
	$(eval NODE="$(shell echo $(NODE)| grep -oE '[^=]+:' | sed 's/://')")
	tmux new-session  "while :; do rsync -e 'ssh -o StrictHostKeyChecking=no' --delete -rzai --chmod=Du+rwx --inplace --exclude='__pycache__' --exclude=".cache" --exclude='.git' --exclude='*.pyc' --exclude='*.swp' --exclude='.idea' . $(NODE):~/mujoco_py/ ; echo -n "."; sleep 1; done" \; \
	     split-window "ssh -t -o StrictHostKeyChecking=no $(NODE) 'cd ~/mujoco_py && export GPUS=$(GPUS) && /bin/bash'; " \; select-layout 9ce0,204x51,0,0[204x4,0,0,32,204x46,0,5,33]


upload:
	rm -rf dist
	python setup.py sdist
	twine upload dist/*

check-license:
ifeq ("","$(wildcard $(MUJOCO_LICENSE_PATH))")
    $(error "License key not found at location $(MUJOCO_LICENSE_PATH). Use MUJOCO_LICENSE_PATH to specify its path")
endif
