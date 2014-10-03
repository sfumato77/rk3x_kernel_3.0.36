#!/bin/sh
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=/home/sanek/build/toolchain/arm-eabi-4.6/bin/arm-eabi-
KERNEL_IN=~/build/surf1011_kernel_3.0.36/arch/arm/boot
KERNEL_OUT=~/build/rk-tools/boot.img.dump
echo '===================================================================='
echo "ARCHITECTURE    = $ARCH"
echo "SUBARCHITECTURE = $SUBARCH"
echo "CROSS COMPILER  = $CROSS_COMPILE"
echo '===================================================================='
make clean &&
make -j4 &&
cp $KERNEL_IN/Image $KERNEL_OUT/kernel &&
cd ~/build/rk-tools &&
./imgrepackerrk boot.img.cfg &&
rm kernel.zip &&
cp boot.img kernel/boot.img &&
cd kernel &&
zip -r -9 kernel.zip META-INF boot.img &&
mv kernel.zip ../
echo '========================DONE======================================='