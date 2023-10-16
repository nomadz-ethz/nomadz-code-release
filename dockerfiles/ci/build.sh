#!/bin/bash

cd $(dirname -- "$0")

docker build -f Dockerfile -t ghcr.io/nomadz-ethz/nomadz-ci .
