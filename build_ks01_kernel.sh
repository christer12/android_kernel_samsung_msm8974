#!/bin/bash

MODEL=$1
# ks01skt, ks01ktt, ks01lgt

VER=$2
#03

BUILD_TOP_DIR=$(pwd)

KERNEL_DEFCONFIG=msm8974_sec_defconfig
VARIANT_DEFCONFIG=msm8974_sec_${MODEL}_rev${VER}_defconfig

KERNEL_ZIMG=$BUILD_TOP_DIR/arch/arm/boot/zImage

echo "build config="$KERNEL_DEFCONFIG VARIANT_DEFCONFIG=$VARIANT_DEFCONFIG" "

export ARCH=arm
export CROSS_COMPILE=/opt/toolchains/arm-eabi-4.6/bin/arm-eabi-
#export CROSS_COMPILE=~/Toolchain/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-

make $KERNEL_DEFCONFIG VARIANT_DEFCONFIG=$VARIANT_DEFCONFIG
make

cp -rf $KERNEL_ZIMG $BUILD_TOP_DIR