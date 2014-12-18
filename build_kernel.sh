#!/bin/bash

BUILD_TOP_DIR=$(pwd)

export ARCH=arm
export CROSS_COMPILE=/opt/toolchains/arm-eabi-4.6/bin/arm-eabi-

make msm8974_sec_defconfig VARIANT_DEFCONFIG=msm8974_sec_ks01skt_rev03_defconfig SELINUX_DEFCONFIG=selinux_defconfig
make