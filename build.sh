#!/bin/bash

printf "Phobis build script\n\n"

if [[ $EUID -ne 0 ]]; then
printf "This script requires root privileges. Run with 'sudo' or as root.\n"
exit 1
fi

# Backup data to a "phobis.old" file.
printf "All data previously stored in 'phobis.img' will be lost!\n"
printf "Backup will be made.\n"
rm phobis.old 2> /dev/null
mv phobis.img phobis.old 2> /dev/null

printf "Assembling bootloader...\n"
nasm boot/bootloader.asm -f bin -o phobis.img

printf "Assembling kernel...\n"
nasm kernel/kernel.asm -f bin -o kernel.bin

printf "Installing kernel...\n"
cat kernel.bin >> phobis.img
rm kernel.bin

printf "Expanding image...\n"
dd bs=512 count=2812 if=/dev/zero >> phobis.img 2> /dev/null

printf "Creating temporary folder to store binaries...\n"
mkdir tmp

printf "Assembling programs...\n"
for asm_file in programs/*.asm
do
	base_name=${asm_file%.asm}
	base_name=${base_name:8}
	printf "Assembling '$asm_file'...\n"
    nasm "$asm_file" -f bin -o "tmp/${base_name}.com"
done

printf "Creating mount point for image...\n"
mkdir mnt

printf "Mounting image...\n"

if [[ "`uname`" == "Linux" ]]; then
mount phobis.img ./mnt
fi

if [[ "`uname`" == "FreeBSD" ]]; then
mdconfig -a -t vnode -f phobis.img -u 0
mount_msdosfs /dev/md0 ./mnt
fi

printf "Copying files to image...\n"
cp -r extra/* mnt/ 2> /dev/null
cp tmp/* mnt/

printf "Unmounting image...\n"
sync
umount ./mnt

if [[ `uname` == "FreeBSD" ]]; then
mdconfig -du md0
fi

printf "Cleaning up...\n"
rm -rf tmp
rm -rf mnt

printf "Done!\n\n"
printf "If everything executed correctly, a file named 'phobis.img'\n"
printf "should have been built.\n"

exit 0
