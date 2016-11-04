FROM ubergarm/armhf-ubuntu:trusty

LABEL description="This repo is always the latest image for the rowboat1 AUV system \
whose software may be found at https://github.com/Seanmatthews/rowboat1. The image is \
based upon ubergarm/armhf-ubuntu:trusty. It runs on an Odroid XU4 (ARM) computer."

RUN apt-get update

# CMD ["/bin/bash"]

# RUN apt-get update && apt-get install -y --no-install-recommends \
#        git \
#	&& rm -rf /var/lib/apt/lists/*



