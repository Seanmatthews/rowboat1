#!/bin/bash
set -e
docker login -e=$DOCKER_EMAIL -u=$DOCKER_USERNAME -p=$DOCKER_PASSWORD
docker images
docker push rowboat/rowboat1
rm -rf ~/.docker || rm -rf ~/.dockercng || true
