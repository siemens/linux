#!/bin/sh

#
## parameters:
#
## 1. kernel image
## 2. root_fs
## 3. endianess: big_endian | little_endian
## 3. output directory
##
## example: ./cr_jffs2_img_arm_gnueabi.sh /tftpboot/uImage /tftpboot/rootfs_arm-gnueabi [big_endian | little_endian ] /tftpboot/<user>

usage_exit() {
	echo
	echo "Usage: cr_jffs2_img_arm_gnueabi.sh <kernel_image> <rootfs_dir> <endianess> <output_dir>"
	echo "Example: ./cr_jffs2_img_arm_gnueabi.sh /tftpboot/uImage /tftpboot/rootfs_arm-gnueabi-be big_endian  /tftpboot<user>"
	echo
	exit
}

KERNEL=$1
ROOT_FS=$2
ENDIANESS=$3
OUTPUT_DIR=$4

if [ ! -f "${KERNEL}" ]; then
	echo
	echo kernel missing or incorrect
	usage_exit
fi

if [ ! -d "${ROOT_FS}" ]; then
	echo
	echo rootfs_dir missing or incorrect
	usage_exit
fi
if [ "${ENDIANESS}" = "big_endian" ]; then
	endianess="-b"
elif [ "${ENDIANESS}" = "little_endian" ]; then
	endianess="-l"
else
	echo endianess missing or incorrect
	usage_exit
fi

if [ ! -d "${OUTPUT_DIR}" ]; then
	echo
	echo output_dir missing or incorrect
	usage_exit
fi

IMAGE=${OUTPUT_DIR}/jffs2_arm.image
TEMP_IMAGE=${OUTPUT_DIR}/temp_image

rm -f ${IMAGE} ${OUTPUT_DIR}/temp_image ${OUTPUT_DIR}/temp_kernel

mkfs.jffs2 --eraseblock=16KiB ${endianess} -p -n -d ${ROOT_FS} -o ${TEMP_IMAGE}

bzip2 -c ${KERNEL} > ${IMAGE}
bzip2 -c ${TEMP_IMAGE} >> ${IMAGE}

rm ${TEMP_IMAGE}
echo
echo "file ${IMAGE} is ready"
echo "Use the mtdburn command to burn it to flash "
echo



