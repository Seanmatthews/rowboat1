FROM ubergarm/armhf-ubuntu:trusty

ENV QEMU_EXECVE 1

COPY . /usr/bin

# COPY sh-shim /usr/bin
# COPY qemu-arm-static /usr/bin
# COPY cross-build-start /usr/bin
# COPY cross-build-end /usr/bin


RUN ["cross-build-start"]
RUN apt-get update
RUN ["cross-build-end"]


