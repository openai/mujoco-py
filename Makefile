.PHONY: all clean build test mount_shell shell upload check-env

MUJOCO_LICENSE_PATH ?= ~/.mujoco/mjkey.txt

all: test

clean:
	rm -rf mujoco_py.egg-info
	rm -rf */__pycache__
	rm -rf */*/__pycache__
	rm -rf mujoco_py/generated/_pyxbld*
	rm -rf mujoco_py/generated_cymj*
	rm -rf mujoco_py/cythonlock_*.pyc
	rm -rf dist
	rm -rf build

build: check-license
	cp $(MUJOCO_LICENSE_PATH) mjkey.txt
	docker build -t mujoco_py . || rm mjkey.txt && rm mjkey.txt

test: build
	# run it interactive mode so we can abort with CTRL+C
	docker run --rm -i mujoco_py pytest

mount_shell:
	docker run --rm -it -v `pwd`:/dev mujoco_py /bin/bash -c "pip uninstall -y mujoco_py; rm -rf /mujoco_py; (cd /dev; /bin/bash)"

shell:
	docker run --rm -it mujoco_py /bin/bash

upload:
	rm -rf dist
	python setup.py sdist
	twine upload dist/*

check-license:
ifeq ("","$(wildcard $(MUJOCO_LICENSE_PATH))")
    $(error "License key not found at location $(MUJOCO_LICENSE_PATH). Use MUJOCO_LICENSE_PATH to specify its path")
endif
