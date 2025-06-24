# Copyright (C) 2024 OpenDLV

FROM ubuntu:22.04 as builder
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt-get install -y \
    build-essential \
    cmake \
    software-properties-common \
    libopencv-dev

RUN add-apt-repository 'ppa:chrberger/libcluon' && \
    apt-get update && \
    apt-get install libcluon

ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install


FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt-get install -y \
    libopencv-core-dev \
    libopencv-imgproc-dev \
    libopencv-video-dev \
    libopencv-calib3d-dev \
    libopencv-features2d-dev \
    libopencv-objdetect-dev \
    libopencv-highgui-dev \
    libopencv-videoio-dev \
    libopencv-flann-dev\
    python3-opencv

WORKDIR /usr/bin
COPY --from=builder /tmp/dest /usr
ENTRYPOINT ["/usr/bin/opendlv-perception-helloworld"]
