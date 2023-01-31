FROM ubuntu:16.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -qq \
    && apt-get -y install vim git wget\
    build-essential bison libc6-dev-i386 gcc-multilib g++-multilib byacc \
    autoconf m4 automake gettext autopoint libtool
#     && rm -rf /var/lib/apt/lists/*

WORKDIR /home/killah/Documents/master_thesis/mission_planning/DiNo 

# install old flex
RUN wget https://github.com/westes/flex/releases/download/flex-2.5.39/flex-2.5.39.tar.gz \
    && tar -xf flex-2.5.39.tar.gz 

RUN cd flex-2.5.39 \
    && ./autogen.sh \
    && ./configure && make && make install

# get dino
RUN git clone https://github.com/KCL-Planning/DiNo

RUN cd DiNo/src && make

# COPY . .
