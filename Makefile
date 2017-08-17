.PHONY: all clean build test mount_shell shell upload check-env

MUJOCO_LICENSE_PATH ?= ~/.mujoco/mjkey.txt
DOCKER_NAME ?= mujoco_py

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

test: build
	# run it interactive mode so we can abort with CTRL+C
	docker run --rm -i $(DOCKER_NAME) pytest

test_gpu:
	nvidia-docker run --rm -i $(DOCKER_NAME) pytest -m "not requires_glfw"

mount_shell:
	docker run --rm -it -v `pwd`:/code $(DOCKER_NAME) /bin/bash -c "pip3 uninstall -y mujoco_py; rm -rf /mujoco_py; (cd /code; /bin/bash)"

shell:
	docker run --rm -it $(DOCKER_NAME) /bin/bash

upload:
	rm -rf dist
	python setup.py sdist
	twine upload dist/*

check-license:
ifeq ("","$(wildcard $(MUJOCO_LICENSE_PATH))")
    $(error "License key not found at location $(MUJOCO_LICENSE_PATH). Use MUJOCO_LICENSE_PATH to specify its path")
endif
