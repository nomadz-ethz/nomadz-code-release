set -e
set -o pipefail

# parameters
if [ "$#" -lt 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage: $0 image.opn output.ext3"
    exit 1
fi
INPUT_IMAGE="$1"
OUTPUT_IMAGE="$2"

# .opn image offsets
HEADER_SIZE=4096
INSTALLER_OFFSET=$HEADER_SIZE
INSTALLER_SIZE=1048576
IMAGE_OFFSET=$(($INSTALLER_SIZE+$INSTALLER_OFFSET))

# for dd bs
BLOCK_SIZE=4096

# extract filesystem and installer from .opn image
echo "Extract filesystem..."
dd if="$INPUT_IMAGE" of="${OUTPUT_IMAGE}.gz" skip=$(($IMAGE_OFFSET/$BLOCK_SIZE)) bs=$BLOCK_SIZE
sed -i '$ s/\x00*$//' ./${OUTPUT_IMAGE}.gz # remove zeros
echo "Done!"
echo "Decompress filesystem..."
pigz -df "${OUTPUT_IMAGE}.gz"
rm -f ${OUTPUT_IMAGE}.gz
echo "Done!"
