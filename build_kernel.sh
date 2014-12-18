#!/bin/bash

MODEL=$1
# ks01skt, ks01ktt, ks01lgt

#03

BUILD_TOP_DIR=$(pwd)

KERNEL_ZIMG=$BUILD_TOP_DIR/arch/arm/boot/zImage

export ARCH=arm
export CROSS_COMPILE=/opt/toolchains/arm-eabi-4.6/bin/arm-eabi-

make msm8974_sec_defconfig VARIANT_DEFCONFIG=msm8974_sec_ks01skt_rev03_defconfig SELINUX_DEFCONFIG=selinux_defconfig SELINUX_LOG_DEFCONFIG=selinux_log_defconfig TIMA_DEFCONFIG=tima_defconfig
make

cp -rf $KERNEL_ZIMG $BUILD_TOP_DIR