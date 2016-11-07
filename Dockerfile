FROM ubergarm/armhf-ubuntu:trusty

ENV QEMU_EXECVE 1
# Force armv6l
# ENV QEMU_CPU arm1176

COPY . /usr/bin

RUN [ "cross-build-start" ]

# Packages
RUN apt-get update

RUN [ "cross-build-end" ]


