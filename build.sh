#!/bin/sh
set -e

for ARCH in aarch64 armv7hf; do
    echo "--- Building for $ARCH ---"
    docker build --progress=plain --no-cache --build-arg ARCH=$ARCH --tag dataq-$ARCH .
    CONTAINER=$(docker create dataq-$ARCH)
    docker cp ${CONTAINER}:/opt/app ./build-$ARCH
    docker rm ${CONTAINER}
    mv build-$ARCH/*.eap .
    rm -rf build-$ARCH
    echo "--- Done $ARCH ---"
done

ls -lh *.eap
