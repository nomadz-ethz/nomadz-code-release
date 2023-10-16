# syntax=docker/dockerfile:1

# build opencv
FROM ubuntu:focal AS build-opencv
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
    wget \
    unzip \
    clang \
    cmake \
    ninja-build
WORKDIR /root

# build opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.2.zip
RUN unzip opencv.zip && mv opencv-4.5.2 opencv
RUN cmake -B opencv/build -S opencv -G Ninja -DCMAKE_INSTALL_PREFIX=/opt -DCMAKE_BUILD_TYPE=Release -DBUILD_LIST=core,imgproc,highgui,videoio
RUN cmake --build opencv/build -j $(nproc)
RUN cmake --install opencv/build

# build the final image
FROM ros:galactic-ros-base-focal AS install-deps

# install system dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update
RUN apt install -y \
    ninja-build \
    ccache \
    cmake \
    qtbase5-dev \
    libqt5svg5-dev \
    qttools5-dev \
    libglew-dev \
    libgtk-3-dev \
    libjpeg62-dev \
    libasound2-dev \
    libharfbuzz-dev \
    libfreetype6-dev \
    libatlas-base-dev \
    gfortran \
    libmsgpack-dev \
    liblapack-dev \
    libboost-filesystem-dev \
    libxml2-dev

FROM install-deps as final

# copy opencv headers and binaries
COPY --from=build-opencv /opt/ /usr/local/

# create nomadz user with sudo, mostly for debugging
RUN useradd -m -s /bin/bash -N -u 1000 nomadz && \
    echo "nomadz ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd 

USER root
WORKDIR /root
