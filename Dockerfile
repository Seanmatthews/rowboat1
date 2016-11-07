FROM ubergarm/armhf-ubuntu:trusty

ENV QEMU_EXECVE 1

COPY qemu-arm-static /usr/bin

RUN ["cross-build-start"]
RUN apt-get update
RUN ["cross-build-end"]


