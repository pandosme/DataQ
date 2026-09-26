#!/bin/sh
set -e

docker build --progress=plain --no-cache --build-arg ARCH=aarch64 --tag dataq-ddh-aarch64 .
docker cp $(docker create dataq-ddh-aarch64):/opt/app ./build
mv build/*.eap .
rm -rf build
docker build --progress=plain --no-cache --tag dataq-ddh-armv7hf .
docker cp $(docker create dataq-ddh-armv7hf):/opt/app ./build
mv build/*.eap .
rm -rf build
