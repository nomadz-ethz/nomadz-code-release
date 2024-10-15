#!/bin/bash

# check if cmake-format is installed
if ! command -v cmake-format &> /dev/null
then
    echo "cmake-format is not installed. Please install it with pip or your system package manager."
    exit 1
fi

# cd to the root of the project
cd $(realpath $(dirname -- "$( readlink -f -- "$0"; )")/../..)

all_files=$(git ls-files **/CMakeLists.txt)
echo "$all_files"
cmake-format -c ./.cmake-format -i ${all_files}

# check if git diff is dirty (i.e. if clang-format changed something) before
# exiting, because if it is, the exit code will be 1
# this behavior is desired, because it will cause the CI to fail
git diff --quiet --exit-code ${all_files}
