#!/usr/bin/env bash
containers=$(docker ps -f label=roomba.arduino -q)
images=$(docker images -f label=roomba.arduino -q)

docker stop $containers || true
docker rm -f $containers || true
docker rmi -f $images || true