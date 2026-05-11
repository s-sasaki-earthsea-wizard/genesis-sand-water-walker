# genesis-sand-water-walker — Makefile
#
# Build a Genesis-based Docker image and run the soft-terrain dive simulations
# (humanoid into sand, planar walker into water). See `make help`.

# ----- configurable ---------------------------------------------------------
IMAGE         ?= genesis-sand-water-walker:latest
DOCKERFILE    ?= docker/Dockerfile
CONTAINER     ?= gswwalker-run
GAIT_HZ       ?= 1.0
KNEE_AMPLITUDE?= 0.6
PROJECT_DIR   := $(shell pwd)
WORKSPACE     ?= /workspace
DOCKER_RUN    ?= docker run --rm --gpus all \
                   -e PYTHONUNBUFFERED=1 \
                   -v $(PROJECT_DIR):$(WORKSPACE) \
                   -w $(WORKSPACE) \
                   --entrypoint /bin/bash

# ----- targets --------------------------------------------------------------
.DEFAULT_GOAL := help

.PHONY: help build dive-humanoid dive-walker dive-all march-walker shell \
        clean clean-outputs clean-image check-gpu

## help: list available targets
help:
	@echo "genesis-sand-water-walker — available targets"
	@echo ""
	@IMAGE="$(IMAGE)" CONTAINER="$(CONTAINER)" awk \
		'BEGIN{FS=":"} /^## /{sub(/^## /,"",$$0); gsub(/\$$\{IMAGE\}/,ENVIRON["IMAGE"]); gsub(/\$$\{CONTAINER\}/,ENVIRON["CONTAINER"]); printf "  %s\n", $$0}' \
		$(MAKEFILE_LIST)
	@echo ""
	@echo "Variables you can override on the command line:"
	@echo "  IMAGE=$(IMAGE)"
	@echo "  CONTAINER=$(CONTAINER)"

## build: build the Docker image (tag: ${IMAGE})
build:
	docker build -t $(IMAGE) -f $(DOCKERFILE) docker/

## check-gpu: quick sanity check that the host can run a GPU container
check-gpu:
	docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi || \
		echo "GPU check failed. Make sure the NVIDIA Container Toolkit is installed."

## dive-humanoid: simulate the humanoid diving into a sand pool (~6 min on RTX 5080)
dive-humanoid:
	$(DOCKER_RUN) $(IMAGE) -c "python -u scripts/humanoid_on_sand.py"

## dive-walker: simulate the planar walker diving into a water pool (~6 min on RTX 5080)
dive-walker:
	$(DOCKER_RUN) $(IMAGE) -c "python -u scripts/walker_on_water.py"

## dive-all: run both dive simulations sequentially
dive-all: dive-humanoid dive-walker

## march-walker: walker marches in place on a flat rigid floor (no sand/water)
##   override GAIT_HZ / KNEE_AMPLITUDE on the command line, e.g.
##   `make march-walker GAIT_HZ=2.0 KNEE_AMPLITUDE=0.9`
march-walker:
	$(DOCKER_RUN) $(IMAGE) -c "python -u scripts/walker_marching.py --gait-hz $(GAIT_HZ) --knee-amplitude $(KNEE_AMPLITUDE)"

## shell: open an interactive bash shell inside the Genesis container
shell:
	$(DOCKER_RUN) -it $(IMAGE)

## clean-outputs: remove generated videos and CSVs in outputs/
clean-outputs:
	rm -rf outputs/

## clean-image: remove the built Docker image
clean-image:
	-docker image rm $(IMAGE)

## clean: clean-outputs + clean-image
clean: clean-outputs clean-image
