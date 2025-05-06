#!/bin/sh

# Script should be run from the src/ folder ("sh .docker/run.sh")

docker image build -t roarm-image $PWD/.docker
