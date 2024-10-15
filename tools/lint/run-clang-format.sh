#!/bin/bash

# check if clang-format is installed
if ! command -v clang-format-14 &> /dev/null
then
    echo "clang-format-14 is not installed. Please install it with your system package manager."
    exit 1
fi

# cd to root of the project
cd $(realpath $(dirname -- "$( readlink -f -- "$0"; )")/../..)

exclude_list=""
all_files=$(git ls-files *.h *.hpp *.c *.cpp)

for exclude in $exclude_list
do
    all_files=$(echo "$all_files" | sed "/$exclude/d")
done

echo "$all_files"
clang-format-14 -i ${all_files}

# check if git diff is dirty (i.e. if clang-format changed something) before
# exiting, because if it is, the exit code will be 1
# this behavior is desired, because it will cause the CI to fail
git diff --quiet --exit-code ${all_files}
